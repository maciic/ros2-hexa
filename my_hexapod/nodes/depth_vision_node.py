import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.depth_pub = self.create_publisher(Image, 'depth/image_raw', 1)
        
        self.is_processing = False
        self.is_ready = False # <-- BIZTONSÁGI ZÁR
        self.model_w = 224    # <-- ALAPÉRTELMEZETT ÉRTÉK
        self.model_h = 224    # <-- ALAPÉRTELMEZETT ÉRTÉK
        
        self.stack = ExitStack()
        
        self.get_logger().info("🌊 Hailo DepthAnything AI indítása...")
        
        try:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            hef_path = os.path.join(script_dir, "..", "depth_anything_vits.hef")
            
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
            
            shape = self.hef.get_input_vstream_infos()[0].shape
            if len(shape) == 4:
                self.model_h, self.model_w = shape[1], shape[2]
            elif len(shape) == 3:
                self.model_h, self.model_w = shape[0], shape[1]
                
            self.is_ready = True # <-- SIKERES INDULÁS, KINYITJUK A ZÁRAT!
            
            self.smooth_d_min = None
            self.smooth_d_max = None
            
            self.get_logger().info(f"✅ TÉRBELI LÁTÁS AKTÍV! ({self.model_w}x{self.model_h})")
            
        except Exception as e:
            # HA ITT ELHASAL, LÁTNI FOGJUK A PONTOS OKOT:
            self.get_logger().error(f"❌ HAILO MÉLYSÉG INICIALIZÁLÁSI HIBA: {e}")
            self.stack.close()

    def image_callback(self, msg):
        # HA NEM SIKERÜLT AZ INDULÁS, ELDOBJUK A KÉPET!
        if not self.is_ready or self.is_processing:
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
            
            current_min = raw_depth.min()
            current_max = raw_depth.max()
            
            # --- ÚJ: EXPONENCIÁLIS MOZGÓÁTLAG (Időbeli simítás) ---
            # Ez megakadályozza, hogy a falak "ugráljanak" előre-hátra, ha megváltozik a látótér!
            if self.smooth_d_min is None:
                self.smooth_d_min = current_min
                self.smooth_d_max = current_max
            else:
                alpha = 0.05 # Mennyire kövesse gyorsan a változást? (Kisebb = stabilabb, de lassabb)
                self.smooth_d_min = (1.0 - alpha) * self.smooth_d_min + alpha * current_min
                self.smooth_d_max = (1.0 - alpha) * self.smooth_d_max + alpha * current_max
            
            # Normalizálás a simított értékekkel
            norm_depth = (raw_depth - self.smooth_d_min) / (self.smooth_d_max - self.smooth_d_min + 1e-6)
            # BIZTONSÁGI VÁGÁS: Megakadályozza az uint16 átfordulást (65 méteres tüskéket)!
            norm_depth = np.clip(norm_depth, 0.0, 1.0)
            
            min_dist_mm = 200.0  # 20 centinél közelebbi dolgokat levágja
            max_dist_mm = 4000.0 # 4 méterre nyomja össze a teret (beltérre tökéletes)
            pseudo_metric_depth = (1.0 - norm_depth) * (max_dist_mm - min_dist_mm) + min_dist_mm
            
            depth_resized = cv2.resize(pseudo_metric_depth, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
            depth_16u = depth_resized.astype(np.uint16)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_16u, encoding="16UC1")
            depth_msg.header = msg.header 
            self.depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f"Hiba a callback-ben: {e}")
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