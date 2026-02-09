import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from pymycobot.elephantrobot import ElephantRobot, JogMode
from scipy.spatial.transform import Rotation as R
import time

# --- 基礎工具函數 ---
def transform_point(T, P):
    """ 使用 4x4 矩陣轉換 3D 點 """
    P_h = np.ones(4)
    P_h[:3] = P
    P_transformed = T @ P_h
    return P_transformed[:3]

class StoneGrabSystem:
    def __init__(self):
        # 1. 參數設定
        self.model_path = "best.pt"
        self.robot_ip = "192.168.50.123"
        self.robot_port = 5001
        self.conf_threshold = 0.9
        self.move_speed = 1000        
        self.arrival_timeout = 30    
        self.home_angles = [-10, -85, 90, -95, -90, -40]

        print("=" * 60)
        print(f"🚀 啟動系統... 速度: {self.move_speed} | 信心度: {self.conf_threshold}")
        
        # 2. 初始化 RealSense
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.profile = self.pipeline.start(config)
            self.align = rs.align(rs.stream.color)
            self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            print("✓ RealSense 相機初始化成功")
        except Exception as e:
            print(f"❌ 相機啟動失敗: {e}")

        # 3. 載入模型與標定矩陣
        self.model = YOLO(self.model_path)
        self.T_camera_gripper = np.array([
            [-0.92763612,  0.37336914, -0.0093117,   0.01855558],
            [-0.37346284, -0.92701915,  0.03407355,  0.03314082],
            [ 0.00408989,  0.03508543,  0.99937595,  0.04703275],
            [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]
        ])
        self.tcp_offset = np.array([0, 0, -120]) / 1000.0 

        # 4. 連接機械手臂
        try:
            self.robot = ElephantRobot(self.robot_ip, self.robot_port)
            self.robot.start_client()
            time.sleep(1) # 給予連線緩衝
            print("✓ 機械手臂連線成功")
        except Exception as e:
            print(f"❌ 手臂連線失敗: {e}")
            self.robot = None

        self.latest_object = None
        self.running = False

    def wait_arrival(self, target, mode="coords"):
        """ 改良版等待函數，增加穩定性 """
        start_t = time.time()
        check_mode = JogMode.JOG_TELEOP if mode == "coords" else JogMode.JOG_JOINT
        
        time.sleep(0.3) # 指令發送後的物理反應延遲
        while time.time() - start_t < self.arrival_timeout:
            try:
                if self.robot.is_in_position(target, check_mode):
                    actual = self.robot.get_coords() if mode == "coords" else self.robot.get_angles()
                    print(f"   ✓ 到達目標位置")
                    return True
            except:
                pass 
            time.sleep(0.2)
        print(f"   ⚠️ {mode} 移動逾時")
        return False

    def control_gripper(self, open_gr):
        """ 夾爪控制 """
        if self.robot:
            state = 0 if open_gr else 1
            self.robot.set_digital_out(16, state)
            self.robot.set_digital_out(17, 1 - state)
            time.sleep(1.0) 

    def get_base_coords(self, pixel_x, pixel_y, depth_frame):
        """ 像素轉基座座標 """
        depth = depth_frame.get_distance(pixel_x, pixel_y)
        if depth == 0 or self.robot is None: return None
        
        cam_p = rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel_x, pixel_y], depth)
        curr = self.robot.get_coords()
        if not curr: return None
        
        xyz, rpy = np.array(curr[:3])/1000.0, curr[3:]
        
        T_base_gripper = np.eye(4)
        T_base_gripper[:3, 3] = xyz
        T_base_gripper[:3, :3] = R.from_euler('xyz', rpy, degrees=True).as_matrix()
        
        P_gripper = transform_point(self.T_camera_gripper, cam_p)
        P_base_raw = transform_point(T_base_gripper, P_gripper)
        P_base = P_base_raw - self.tcp_offset 
        
        return P_base * 1000.0

    def grab_process(self):
        """ 核心抓取流程（非阻塞版） """
        if not self.latest_object:
            print("⚠️ 未鎖定目標，取消操作")
            return

        target_xyz = self.latest_object['base_pos']
        print(f"\n📍 目標確定: X:{target_xyz[0]:.1f} Y:{target_xyz[1]:.1f} Z:{target_xyz[2]:.1f}")
        
        # 獲取當前姿態 RPY
        curr_coords = self.robot.get_coords()
        if not curr_coords: 
            print("❌ 無法取得手臂座標")
            return
        curr_rpy = curr_coords[3:]
        
        # 定義路徑點
        up_pos = [target_xyz[0], target_xyz[1], target_xyz[2] + 50, *curr_rpy]
        down_pos = [target_xyz[0], target_xyz[1], target_xyz[2] - 5, *curr_rpy]
        
        try:
            print("Step 1: 移至上方...")
            self.robot.write_coords(up_pos, self.move_speed)
            if not self.wait_arrival(up_pos): return

            print("Step 2: 開爪...")
            self.control_gripper(True)

            print("Step 3: 下降抓取...")
            self.robot.write_coords(down_pos, self.move_speed)
            if not self.wait_arrival(down_pos): return

            print("Step 4: 夾緊...")
            self.control_gripper(False)

            print("Step 5: 提起...")
            lift_pos = [target_xyz[0], target_xyz[1], target_xyz[2] + 100, *curr_rpy]
            self.robot.write_coords(lift_pos, self.move_speed)
            self.wait_arrival(lift_pos)
            self.control_gripper(True)
            print("Step 6: 復位...")
            self.robot.write_angles(self.home_angles, self.move_speed)
            self.wait_arrival(self.home_angles, mode="angles")
            self.control_gripper(False)
            
            print("✨ 任務圓滿完成")
        except Exception as e:
            print(f"❌ 執行中斷: {e}")

    def run(self):
        self.running = True
        print("\n[操作提示]")
        print(" > 按 'g' 鍵：執行自動抓取流程")
        print(" > 按 'q' 鍵：結束程式")
        
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                color_img = np.asanyarray(aligned.get_color_frame().get_data())
                depth_frame = aligned.get_depth_frame()

                # 推論
                results = self.model(color_img, conf=self.conf_threshold, verbose=False)
                
                # 處理偵測結果
                if len(results) > 0 and results[0].masks is not None:
                    # 抓取信心度最高的物件
                    boxes = results[0].boxes.data.cpu().numpy()
                    idx = np.argmax(boxes[:, 4])
                    
                    mask = results[0].masks.data.cpu().numpy()[idx]
                    mask_res = cv2.resize(mask, (640, 480))
                    y_c, x_c = np.where(mask_res > 0.5)
                    
                    if len(x_c) > 0:
                        cx, cy = int(np.mean(x_c)), int(np.mean(y_c))
                        base_p = self.get_base_coords(cx, cy, depth_frame)
                        
                        if base_p is not None:
                            self.latest_object = {'base_pos': base_p}
                            # 視覺輔助
                            cv2.circle(color_img, (cx, cy), 7, (0, 0, 255), -1)
                            cv2.putText(color_img, f"READY: {base_p[0]:.0f}, {base_p[1]:.0f}", 
                                        (cx+15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.imshow('Stone Grabber System', color_img)
                
                # 關鍵：使用 OpenCV 的 waitKey 來處理互動
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('g'):
                    if self.latest_object:
                        self.grab_process()
                    else:
                        print("⚠️ 畫面上沒看到目標物喔！")

        finally:
            print("\n系統關閉中...")
            if self.pipeline:
                self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    system = StoneGrabSystem()
    system.run()