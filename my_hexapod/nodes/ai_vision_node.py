import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from contextlib import ExitStack

from hailo_platform import (HEF, VDevice, HailoStreamInterface, ConfigureParams,
                            InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams)

# Importáljuk az ÚJ Állapotgépet!
from my_hexapod.core.ai_state_machine import AIStateMachine

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ai', 10)
        self.cmd_str_pub = self.create_publisher(String, 'robot_command', 10)
        self.image_pub = self.create_publisher(CompressedImage, 'ai_vision_overlay/compressed', 1)
        
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.mode_sub = self.create_subscription(String, 'ai_current_mode', self.mode_callback, 10)
        
        self.bridge = CvBridge() 
        self.is_processing = False 
        self.current_ai_mode = "STANDBY"
        
        # Példányosítjuk az Állapotgépet
        self.state_machine = AIStateMachine()

        self.get_logger().info("🧠 Hailo-8 POSE AI indítása...")
        self.stack = ExitStack()
        
        try:
            script_dir = os.path.dirname(os.path.realpath(__file__))
            hef_path = os.path.join(script_dir, "..", "yolov8s_pose.hef")
            
            if not os.path.exists(hef_path):
                self.get_logger().error(f"❌ NEM TALÁLOM A POSE HEF FÁJLT: {hef_path}")
                return

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
            
            shape = self.hef.get_input_vstream_infos()[0].shape
            self.model_h, self.model_w = (shape[1], shape[2]) if len(shape) == 4 else (640, 640)
                
            self.get_logger().info(f"✅ LÁTÓIDEG KÉSZEN ÁLL! ({self.model_w}x{self.model_h})")
            
        except Exception as e:
            self.get_logger().error(f"❌ HAILO INICIALIZÁLÁSI HIBA: {e}")
            self.stack.close()

    def mode_callback(self, msg):
        self.current_ai_mode = msg.data
        
        # --- ÚJ: Szelektív amnézia altatáskor ---
        # Ha a robot megáll (STANDBY), azonnal nullázzuk a State Machine memóriáját is!
        if self.current_ai_mode == "STANDBY" and hasattr(self, 'state_machine'):
            self.state_machine.last_mode = "STANDBY"
            self.state_machine._reset_memory()

    def image_callback(self, msg):
        # --- POWER GATING: Ha Standby, azonnal eldobja a képet, a chip pihen! ---
        if self.is_processing or self.current_ai_mode == "STANDBY":
            return
            
        self.is_processing = True

        try:
            frame_original = self.bridge.imgmsg_to_cv2(msg, "rgb8") 
            orig_h, orig_w = frame_original.shape[:2]
            input_img = cv2.resize(frame_original, (self.model_w, self.model_h))
            
            # Csak aktív AI módban ébresztjük fel a Hailo chipet
            infer_results = self.infer_pipeline.infer({self.input_name: np.expand_dims(input_img, axis=0)})
            
            best_person_keypoints = None
            global_max_conf = 0.0

            vstream_infos = self.hef.get_output_vstream_infos()
            grids = {}
            
            for v_info in vstream_infos:
                grid = infer_results[v_info.name][0]
                if len(grid.shape) != 3: continue
                h, w, c = grid.shape
                
                if (h, w) not in grids: grids[(h, w)] = {'cls': None, 'kpts': None, 'unknown_64': []}
                    
                if c <= 8: grids[(h, w)]['cls'] = grid
                elif c == 51 or c == 56: grids[(h, w)]['kpts'] = grid
                elif c == 64: grids[(h, w)]['unknown_64'].append(grid)
                else: grids[(h, w)]['kpts'] = grid
                    
            for (h, w), branches in grids.items():
                if branches['kpts'] is None and len(branches['unknown_64']) >= 2:
                    t1, t2 = branches['unknown_64'][0], branches['unknown_64'][1]
                    branches['kpts'] = t1 if np.max(np.abs(t1[:, :, 51:])) < 1e-5 else t2

            for (h, w), branches in grids.items():
                if branches['cls'] is None or branches['kpts'] is None: continue
                
                cls_grid = branches['cls']
                kpts_grid = branches['kpts']
                scores = cls_grid[:, :, 0] 
                
                cy, cx = np.unravel_index(np.argmax(scores), (h, w))
                local_max_conf = scores[cy, cx]
                
                if local_max_conf > 0.6 and local_max_conf > global_max_conf:
                    global_max_conf = local_max_conf
                    kp_raw = kpts_grid[cy, cx, :51]
                    parsed_keypoints = {}
                    
                    stride = self.model_w / w 
                    scale_x = orig_w / self.model_w
                    scale_y = orig_h / self.model_h
                    
                    for i in range(17):
                        idx = i * 3
                        kx, ky, kconf = kp_raw[idx], kp_raw[idx+1], kp_raw[idx+2]
                        
                        px = kx if abs(kx) > self.model_w else (kx * 2.0 + cx) * stride
                        py = ky if abs(kx) > self.model_w else (ky * 2.0 + cy) * stride
                            
                        parsed_keypoints[i] = {'x': int(px * scale_x), 'y': int(py * scale_y), 'conf': float(kconf)}
                        
                    best_person_keypoints = parsed_keypoints

            # --- ÁTADJUK AZ IRÁNYÍTÁST AZ ÁLLAPOTGÉPNEK ---
            cmd_vel, anim_cmd = self.state_machine.process(self.current_ai_mode, best_person_keypoints)

            # --- VIZUALIZÁCIÓ ---
            do_visualize = self.image_pub.get_subscription_count() > 0
            
            if do_visualize:
                overlay_frame = cv2.cvtColor(frame_original, cv2.COLOR_RGB2BGR)
                cv2.putText(overlay_frame, f"MODE: {self.current_ai_mode} | CONF: {global_max_conf*100:.0f}%", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                if best_person_keypoints is not None:
                    for i, kp in best_person_keypoints.items():
                        if kp['conf'] > 0.4: cv2.circle(overlay_frame, (kp['x'], kp['y']), 5, (0, 0, 255), -1)
                    
                    l_hip, r_hip = best_person_keypoints.get(11), best_person_keypoints.get(12)
                    if l_hip and r_hip and l_hip['conf'] > 0.4 and r_hip['conf'] > 0.4:
                        cv2.line(overlay_frame, (l_hip['x'], l_hip['y']), (r_hip['x'], r_hip['y']), (0, 255, 0), 3)
                        
                success, encoded_image = cv2.imencode('.jpg', overlay_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if success:
                    compressed_msg = CompressedImage()
                    compressed_msg.header = msg.header
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded_image.tobytes()
                    self.image_pub.publish(compressed_msg)

            # --- PARANCSOK KIKÜLDÉSE ---
            self.cmd_pub.publish(cmd_vel)
            if anim_cmd != "NONE":
                msg_str = String()
                # ÚJ: "STOP" helyett "CLEAR_ANIM"-ot küldünk az Agynak!
                msg_str.data = "CLEAR_ANIM" if anim_cmd == "STOP" else f"ANIM_{anim_cmd}"
                self.cmd_str_pub.publish(msg_str)

        except Exception as e:
            self.get_logger().error(f"Hiba a callback-ben: {e}")
            
        finally:
            self.is_processing = False

    def destroy_node(self):
        self.stack.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()