import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from contextlib import ExitStack
from hailo_platform import (HEF, VDevice, HailoStreamInterface, ConfigureParams,
                            InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams)

class DepthVisionNode(Node):
    def __init__(self):
        super().__init__('depth_vision_node')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.mode_sub = self.create_subscription(String, 'vio_current_mode', self.mode_callback, 10)
        self.depth_pub = self.create_publisher(Image, 'depth/image_raw', 1)

        self.hailo_active = False
        self.is_processing = False

        # --- ÚJ: SC-DepthV3 felbontás (Hailo-8 standard) ---
        self.model_w, self.model_h = 320, 256
        self.stack = ExitStack()

        self.get_logger().info("🌊 SC-DepthV3 Node betöltve, várakozás aktiválásra...")

    def mode_callback(self, msg):
        if msg.data == "ACTIVE" and not self.hailo_active:
            self.activate_hailo()
        elif msg.data == "STANDBY" and self.hailo_active:
            self.deactivate_hailo()

    def activate_hailo(self):
        try:
            self.get_logger().info("🚀 Hailo SC-DepthV3 chip ébresztése...")
            script_dir = os.path.dirname(os.path.realpath(__file__))
            # --- ÚJ: Az új HEF fájlneve ---
            hef_path = os.path.join(script_dir, "..", "depth_anything_v2_vits.hef")

            self.hef = HEF(hef_path)
            self.target = self.stack.enter_context(VDevice())

            self.configure_params = ConfigureParams.create_from_hef(hef=self.hef, interface=HailoStreamInterface.PCIe)
            self.network_groups = self.target.configure(self.hef, self.configure_params)
            self.network_group = self.network_groups[0]
            self.network_group_params = self.network_group.create_params()

            self.input_vstreams_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
            self.output_vstreams_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)

            self.infer_pipeline = self.stack.enter_context(
                InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params)
            )
            self.stack.enter_context(self.network_group.activate(self.network_group_params))

            self.input_name = self.hef.get_input_vstream_infos()[0].name
            self.output_name = self.hef.get_output_vstream_infos()[0].name

            self.hailo_active = True
            self.smooth_d_min, self.smooth_d_max = None, None
            self.get_logger().info("✅ SC-DepthV3 AI kész!")
        except Exception as e:
            self.get_logger().error(f"Hiba az aktiváláskor: {e}")

    def deactivate_hailo(self):
        self.get_logger().info("💤 Depth AI altatása...")
        self.stack.close()
        self.hailo_active = False

    def image_callback(self, msg):
        if not self.hailo_active or self.is_processing:
            return

        self.is_processing = True

        try:
            frame_original = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            orig_h, orig_w = frame_original.shape[:2]

            input_img = cv2.resize(frame_original, (self.model_w, self.model_h))

            infer_results = self.infer_pipeline.infer({self.input_name: np.expand_dims(input_img, axis=0)})
            raw_depth = infer_results[self.output_name][0]

            if len(raw_depth.shape) == 3:
                raw_depth = np.squeeze(raw_depth, axis=2)

            # # --- Időbeli simítás (EMA) ---
            # current_min, current_max = raw_depth.min(), raw_depth.max()
            # if self.smooth_d_min is None:
            #     self.smooth_d_min, self.smooth_d_max = current_min, current_max
            # else:
            #     alpha = 0.05
            #     self.smooth_d_min = (1.0 - alpha) * self.smooth_d_min + alpha * current_min
            #     self.smooth_d_max = (1.0 - alpha) * self.smooth_d_max + alpha * current_max

            # # Normalizálás
            # norm_depth = (raw_depth - self.smooth_d_min) / (self.smooth_d_max - self.smooth_d_min + 1e-6)
            # norm_depth = np.clip(norm_depth, 0.0, 1.0)

            # # --- MATEMATIKAI ÁTÁLLÍTÁS ---
            # # Az SC-Depth általában távolságot ad (0 = közel, 1 = távol).
            # # Itt nem vonjuk ki 1.0-ból, hogy a távolabbi pontok nagyobb értéket (mm) kapjanak.
            # min_dist_mm = 200.0
            # max_dist_mm = 10000.0
            # pseudo_metric_depth = norm_depth * (max_dist_mm - min_dist_mm) + min_dist_mm
            
            
            # --- TÖRLENDŐ AZ EDDIGI MIN/MAX ÉS NORMALIZÁLÓ RÉSZ ---
            # ...
            
            # --- ÚJ: DISPARITY (INVERZ) MATEMATIKA ---
            
            # 1. Opcionális: Logoljuk ki a nyers értékeket, hogy lássuk mit ad a Hailo!
            # (Ha már beállítottad, ezt a sort kikommentezheted)
            # self.get_logger().info(f"Nyers Hailo Min: {raw_depth.min():.3f}, Max: {raw_depth.max():.3f}")
            
            # 2. Átalakítás: Távolság (mm) = Konstans / Nyers_Inverz_Érték
            # Mivel a nyers érték nagyon kicsi is lehet, adunk hozzá egy pici számot (1e-4), hogy elkerüljük a nullával osztást.
            FOCAL_CONST = 10000.0 # Ezt az értéket kellhet majd feljebb/lejjebb venni!
            pseudo_metric_depth = FOCAL_CONST / (raw_depth + 1e-4)
            
            # 3. Biztonsági vágás (20 cm-től 8 méterig)
            pseudo_metric_depth = np.clip(pseudo_metric_depth, 200.0, 8000.0)
            
            # 4. Kép átméretezése és publikálása (Ez marad a régi)
            depth_resized = cv2.resize(pseudo_metric_depth, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
            depth_16u = depth_resized.astype(np.uint16)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_16u, encoding="16UC1")
            depth_msg.header = msg.header # Elengedhetetlen az RTAB-Map szinkronhoz!
            self.depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f"Hiba a feldolgozáskor: {e}")
        finally:
            self.is_processing = False

    def destroy_node(self):
        self.stack.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DepthVisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()