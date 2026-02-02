import numpy as np
import cv2
import json
import pyrealsense2 as rs
from pymycobot.elephantrobot import ElephantRobot
from scipy.spatial.transform import Rotation as R


class RealTimeTransformWithTCP:
    """即時相機到機械臂坐標轉換系統 - 整合夾爪偏移和TCP校正偏移並考慮姿態變換"""
    
    def __init__(self, transform_file, tcp_offset_file, gripper_offset_file=None,
                 calibration_pose=[180, 0, 120],
                 target_pose=[90, 20, 90],
                 robot_ip="192.168.50.123", robot_port=5001):
        """
        初始化即時轉換系統
        
        參數:
            transform_file: 轉換矩陣檔案路徑 (JSON格式)
            tcp_offset_file: TCP校正偏移檔案路徑 (JSON格式)
            gripper_offset_file: 夾爪偏移檔案路徑 (TXT格式, 可選)
            calibration_pose: 校正時的姿態角度 [RX, RY, RZ] (度)
            target_pose: 目標姿態角度 [RX, RY, RZ] (度)
            robot_ip: 機械臂IP位址
            robot_port: 機械臂連接埠
        """
        # 載入轉換矩陣和相機參數
        self.load_transformation_matrix(transform_file)
        
        # 載入TCP校正偏移
        self.load_tcp_offset(tcp_offset_file)
        
        # 載入夾爪偏移（可選）
        self.gripper_offset = np.array([0.0, 0.0, 0.0])
        if gripper_offset_file:
            self.load_gripper_offset(gripper_offset_file)
        
        # 儲存姿態資訊
        self.calibration_pose = np.array(calibration_pose)
        self.target_pose = np.array(target_pose)
        
        # 計算轉換後的偏移
        self.transform_tcp_offset()
        
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        
        # ArUco 設定
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.dist_coeffs = np.zeros((5, 1))
        
        # 初始化機械臂連線
        print("\n正在連接機械臂...")
        self.robot = ElephantRobot(robot_ip, robot_port)
        self.robot.start_client()
        print(f"✓ 機械臂已連接: {robot_ip}:{robot_port}")
        
        # 初始化RealSense相機
        print("正在啟動相機...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        print("✓ 相機已啟動")
        
        print("\n" + "="*70)
        print("即時轉換系統已啟動")
        print("="*70)
        print("操作說明:")
        print("  [c] - 獲取當前檢測到的ArUco標記坐標並轉換")
        print("  [m] - 移動機械臂到轉換後的坐標（應用完整TCP偏移）")
        print("  [f] - 移動機械臂到轉換後的坐標（不含TCP偏移）")
        print("  [q] - 退出程式")
        print("="*70)
    
    def load_transformation_matrix(self, filename):
        """載入轉換矩陣"""
        print(f"\n正在載入轉換矩陣: {filename}")
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.transformation_matrix = np.array(data['transformation_matrix'])
        self.rmse_error = data['rmse_error']
        self.camera_matrix = np.array(data['camera_matrix'])
        self.marker_length = data['marker_length']
        
        print(f"✓ 轉換矩陣已載入")
        print(f"  標定點數: {data['calibration_points']}")
        print(f"  RMSE誤差: {self.rmse_error:.2f} mm")
    
    def load_tcp_offset(self, filename):
        """載入TCP校正偏移"""
        print(f"\n正在載入TCP校正偏移: {filename}")
        
        with open(filename, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        # 從offset欄位讀取
        self.tcp_calibration_offset = np.array([
            data['offset']['X'],
            data['offset']['Y'],
            data['offset']['Z']
        ])
        
        print(f"✓ TCP校正偏移已載入")
        print(f"  校正偏移: X={self.tcp_calibration_offset[0]:.3f}, "
              f"Y={self.tcp_calibration_offset[1]:.3f}, "
              f"Z={self.tcp_calibration_offset[2]:.3f} mm")
    
    def load_gripper_offset(self, filename):
        """從txt檔案載入夾爪偏移"""
        import re
        
        print(f"\n正在載入夾爪偏移: {filename}")
        
        with open(filename, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 使用正則表達式提取偏移量
        x_match = re.search(r'X:\s*([-\d.]+)\s*mm', content)
        y_match = re.search(r'Y:\s*([-\d.]+)\s*mm', content)
        z_match = re.search(r'Z:\s*([-\d.]+)\s*mm', content)
        
        if all([x_match, y_match, z_match]):
            self.gripper_offset = np.array([
                float(x_match.group(1)),
                float(y_match.group(1)),
                float(z_match.group(1))
            ])
            print(f"✓ 夾爪偏移已載入")
            print(f"  夾爪偏移: X={self.gripper_offset[0]:.2f}, "
                  f"Y={self.gripper_offset[1]:.2f}, "
                  f"Z={self.gripper_offset[2]:.2f} mm")
            return True
        else:
            print("✗ 無法解析夾爪偏移，使用零偏移")
            return False
    
    def transform_tcp_offset(self):
        """根據姿態變化轉換TCP偏移（包含夾爪偏移和校正偏移）"""
        print(f"\n計算完整TCP偏移轉換:")
        print(f"  校正時姿態: RX={self.calibration_pose[0]:.1f}°, "
              f"RY={self.calibration_pose[1]:.1f}°, "
              f"RZ={self.calibration_pose[2]:.1f}°")
        print(f"  目標姿態:   RX={self.target_pose[0]:.1f}°, "
              f"RY={self.target_pose[1]:.1f}°, "
              f"RZ={self.target_pose[2]:.1f}°")
        
        # 合併夾爪偏移和TCP校正偏移（在校正姿態下）
        self.tcp_offset_original = self.gripper_offset + self.tcp_calibration_offset
        
        print(f"\n原始偏移組成（在校正姿態下）:")
        print(f"  夾爪偏移:   X={self.gripper_offset[0]:7.2f}, "
              f"Y={self.gripper_offset[1]:7.2f}, "
              f"Z={self.gripper_offset[2]:7.2f} mm")
        print(f"  TCP校正:    X={self.tcp_calibration_offset[0]:7.3f}, "
              f"Y={self.tcp_calibration_offset[1]:7.3f}, "
              f"Z={self.tcp_calibration_offset[2]:7.3f} mm")
        print(f"  原始總偏移: X={self.tcp_offset_original[0]:7.2f}, "
              f"Y={self.tcp_offset_original[1]:7.2f}, "
              f"Z={self.tcp_offset_original[2]:7.2f} mm")
        
        # 將歐拉角轉換為旋轉矩陣
        R_calibration = R.from_euler('xyz', self.calibration_pose, degrees=True)
        R_target = R.from_euler('xyz', self.target_pose, degrees=True)
        
        # 計算從校正姿態到目標姿態的相對旋轉
        R_relative = R_target * R_calibration.inv()
        
        # 將TCP偏移向量轉換到新的姿態
        self.tcp_offset_transformed = R_relative.apply(self.tcp_offset_original)
        
        print(f"\n✓ TCP偏移轉換完成:")
        print(f"  轉換後總偏移: X={self.tcp_offset_transformed[0]:7.2f}, "
              f"Y={self.tcp_offset_transformed[1]:7.2f}, "
              f"Z={self.tcp_offset_transformed[2]:7.2f} mm")
        
        # 計算偏移變化
        offset_change = self.tcp_offset_transformed - self.tcp_offset_original
        print(f"  偏移變化:     X={offset_change[0]:7.2f}, "
              f"Y={offset_change[1]:7.2f}, "
              f"Z={offset_change[2]:7.2f} mm")
        
        # 計算偏移向量長度
        original_length = np.linalg.norm(self.tcp_offset_original)
        transformed_length = np.linalg.norm(self.tcp_offset_transformed)
        print(f"\n  原始偏移長度:   {original_length:.2f} mm")
        print(f"  轉換後偏移長度: {transformed_length:.2f} mm")
    
    def camera_to_robot(self, camera_coords):
        """將相機坐標轉換為機械臂坐標（法蘭中心）"""
        # 轉換為齊次坐標
        if len(camera_coords) == 3:
            camera_homogeneous = np.append(camera_coords, 1)
        else:
            camera_homogeneous = camera_coords
        
        # 應用轉換矩陣
        robot_homogeneous = self.transformation_matrix @ camera_homogeneous
        
        return robot_homogeneous[:3]
    
    def camera_to_robot_tcp(self, camera_coords):
        """將相機坐標轉換為機械臂坐標（TCP末端，考慮姿態轉換）"""
        # 先轉換到法蘭中心
        flange_coords = self.camera_to_robot(camera_coords)
        
        # 再加上轉換後的TCP偏移（包含夾爪偏移 + TCP校正偏移）
        tcp_coords = flange_coords + self.tcp_offset_transformed
        
        return tcp_coords
    
    def run(self):
        """執行即時轉換主迴圈"""
        current_robot_target_flange = None
        current_robot_target_tcp = None
        
        try:
            while True:
                # 擷取影像
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                # 檢測 ArUco 標記
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.parameters)
                
                if ids is not None:
                    # 估計姿態
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, 
                        self.camera_matrix, self.dist_coeffs)
                    
                    # 繪製檢測結果
                    for i in range(len(ids)):
                        # 繪製標記邊界
                        cv2.aruco.drawDetectedMarkers(color_image, corners)
                        # 繪製坐標軸
                        cv2.drawFrameAxes(color_image, self.camera_matrix, 
                                        self.dist_coeffs, rvecs[i], tvecs[i], 0.03)
                        
                        # 計算坐標
                        camera_pos = tvecs[i][0] * 1000  # 轉換為 mm
                        flange_pos = self.camera_to_robot(camera_pos)
                        tcp_pos = self.camera_to_robot_tcp(camera_pos)
                        
                        # 顯示資訊
                        x_base = int(corners[i][0][0][0])
                        y_base = int(corners[i][0][0][1])
                        
                        cv2.putText(color_image, f"ID: {ids[i][0]}", 
                                  (x_base, y_base-60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        cv2.putText(color_image, f"Cam: ({camera_pos[0]:.0f}, {camera_pos[1]:.0f}, {camera_pos[2]:.0f})", 
                                  (x_base, y_base-45), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                        
                        cv2.putText(color_image, f"Flange: ({flange_pos[0]:.0f}, {flange_pos[1]:.0f}, {flange_pos[2]:.0f})", 
                                  (x_base, y_base-30), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)
                        
                        cv2.putText(color_image, f"TCP: ({tcp_pos[0]:.0f}, {tcp_pos[1]:.0f}, {tcp_pos[2]:.0f})", 
                                  (x_base, y_base-15), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                
                # 顯示當前機械臂位置
                try:
                    current_robot_pos = self.robot.get_coords()
                    cv2.putText(color_image, f"Current: ({current_robot_pos[0]:.0f}, {current_robot_pos[1]:.0f}, {current_robot_pos[2]:.0f})", 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                except Exception as e:
                    cv2.putText(color_image, "Position unavailable", 
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # 顯示目標位置
                if current_robot_target_tcp is not None:
                    cv2.putText(color_image, f"Target TCP: ({current_robot_target_tcp[0]:.0f}, {current_robot_target_tcp[1]:.0f}, {current_robot_target_tcp[2]:.0f})", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                if current_robot_target_flange is not None:
                    cv2.putText(color_image, f"Target Flange: ({current_robot_target_flange[0]:.0f}, {current_robot_target_flange[1]:.0f}, {current_robot_target_flange[2]:.0f})", 
                              (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
                
                # 顯示TCP偏移（轉換後）
                cv2.putText(color_image, f"Total TCP Offset: ({self.tcp_offset_transformed[0]:.1f}, {self.tcp_offset_transformed[1]:.1f}, {self.tcp_offset_transformed[2]:.1f})", 
                          (10, color_image.shape[0]-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                
                # 顯示目標姿態
                cv2.putText(color_image, f"Target Pose: RX={self.target_pose[0]:.0f} RY={self.target_pose[1]:.0f} RZ={self.target_pose[2]:.0f}", 
                          (10, color_image.shape[0]-60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                
                # 顯示操作提示
                cv2.putText(color_image, "[c]:Capture [m]:Move TCP [f]:Move Flange [q]:Quit", 
                          (10, color_image.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow('Camera to Robot (Gripper+TCP Offset with Pose Transform)', color_image)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("\n退出程式...")
                    break
                    
                elif key == ord('c') and ids is not None:
                    # 獲取並轉換坐標
                    print("\n" + "="*70)
                    print("坐標轉換（含夾爪偏移 + TCP校正偏移 + 姿態轉換）")
                    print("="*70)
                    for i in range(len(ids)):
                        camera_pos = tvecs[i][0] * 1000
                        flange_pos = self.camera_to_robot(camera_pos)
                        tcp_pos = self.camera_to_robot_tcp(camera_pos)
                        
                        current_robot_target_flange = flange_pos
                        current_robot_target_tcp = tcp_pos
                        
                        print(f"ArUco ID: {ids[i][0]}")
                        print(f"  相機坐標:      ({camera_pos[0]:7.1f}, {camera_pos[1]:7.1f}, {camera_pos[2]:7.1f}) mm")
                        print(f"  法蘭中心坐標:  ({flange_pos[0]:7.1f}, {flange_pos[1]:7.1f}, {flange_pos[2]:7.1f}) mm")
                        print(f"  TCP末端坐標:   ({tcp_pos[0]:7.1f}, {tcp_pos[1]:7.1f}, {tcp_pos[2]:7.1f}) mm")
                        print(f"  完整TCP偏移:   ({self.tcp_offset_transformed[0]:7.2f}, "
                              f"{self.tcp_offset_transformed[1]:7.2f}, "
                              f"{self.tcp_offset_transformed[2]:7.2f}) mm")
                        print(f"    - 夾爪偏移(原): ({self.gripper_offset[0]:7.2f}, "
                              f"{self.gripper_offset[1]:7.2f}, "
                              f"{self.gripper_offset[2]:7.2f}) mm")
                        print(f"    - TCP校正(原):  ({self.tcp_calibration_offset[0]:7.3f}, "
                              f"{self.tcp_calibration_offset[1]:7.3f}, "
                              f"{self.tcp_calibration_offset[2]:7.3f}) mm")
                        print("-" * 70)
                
                elif key == ord('m') and current_robot_target_tcp is not None:
                    # 移動機械臂到TCP末端位置
                    print(f"\n移動機械臂到TCP末端位置: "
                          f"({current_robot_target_tcp[0]:.1f}, "
                          f"{current_robot_target_tcp[1]:.1f}, "
                          f"{current_robot_target_tcp[2]:.1f}) mm")
                    print(f"目標姿態: RX={self.target_pose[0]:.1f}°, "
                          f"RY={self.target_pose[1]:.1f}°, "
                          f"RZ={self.target_pose[2]:.1f}°")
                    
                    try:
                        # 使用目標姿態
                        target_coords = [
                            current_robot_target_tcp[0], 
                            current_robot_target_tcp[1], 
                            current_robot_target_tcp[2],
                            self.target_pose[0],
                            self.target_pose[1],
                            self.target_pose[2]
                        ]
                        
                        self.robot.write_coords(target_coords, 1000)
                        print("✓ 移動指令已發送（TCP末端位置 + 目標姿態）")
                    except Exception as e:
                        print(f"✗ 移動失敗: {e}")
                
                elif key == ord('f') and current_robot_target_flange is not None:
                    # 移動機械臂到法蘭中心位置（不含TCP偏移）
                    print(f"\n移動機械臂到法蘭中心位置: "
                          f"({current_robot_target_flange[0]:.1f}, "
                          f"{current_robot_target_flange[1]:.1f}, "
                          f"{current_robot_target_flange[2]:.1f}) mm")
                    
                    try:
                        # 使用目標姿態
                        target_coords = [
                            current_robot_target_flange[0], 
                            current_robot_target_flange[1], 
                            current_robot_target_flange[2],
                            self.target_pose[0],
                            self.target_pose[1],
                            self.target_pose[2]
                        ]
                        
                        self.robot.write_coords(target_coords, 1000)
                        print("✓ 移動指令已發送（法蘭中心位置 + 目標姿態）")
                    except Exception as e:
                        print(f"✗ 移動失敗: {e}")
                
                elif key == ord('c') and ids is None:
                    print("\n未檢測到ArUco標記!")
                
                elif key == ord('m') and current_robot_target_tcp is None:
                    print("\n請先按 'c' 獲取目標坐標!")
                
                elif key == ord('f') and current_robot_target_flange is None:
                    print("\n請先按 'c' 獲取目標坐標!")
        
        except KeyboardInterrupt:
            print("\n程式被中斷")
        
        finally:
            print("正在關閉系統...")
            self.pipeline.stop()
            cv2.destroyAllWindows()
            print("系統已關閉")


def main():
    """主程式"""
    print("="*70)
    print("相機到機械臂即時坐標轉換系統")
    print("含夾爪偏移 + TCP校正偏移 + 姿態轉換")
    print("="*70)
    
    # 設定檔案路徑
    transform_file = 'camera_to_robot_transform.json'
    tcp_offset_file = 'tcp_offset.json'
    gripper_offset_file = 'tcp_stage1_results.txt'  # 夾爪偏移檔案
    
    # 設定姿態
    calibration_pose = [180, 0, 120]  # TCP校正時的姿態
    target_pose = [90, 20, 90]        # 目標使用姿態
    
    # 設定機械臂IP和端口
    robot_ip = "192.168.50.123"
    robot_port = 5001
    
    print(f"\n姿態設定:")
    print(f"  校正時姿態: RX={calibration_pose[0]}°, RY={calibration_pose[1]}°, RZ={calibration_pose[2]}°")
    print(f"  目標姿態:   RX={target_pose[0]}°, RY={target_pose[1]}°, RZ={target_pose[2]}°")
    
    try:
        # 啟動即時轉換系統
        real_time_system = RealTimeTransformWithTCP(
            transform_file=transform_file,
            tcp_offset_file=tcp_offset_file,
            gripper_offset_file=gripper_offset_file,
            calibration_pose=calibration_pose,
            target_pose=target_pose,
            robot_ip=robot_ip,
            robot_port=robot_port
        )
        
        # 執行即時轉換
        real_time_system.run()
        
    except FileNotFoundError as e:
        print(f"\n錯誤: 找不到檔案")
        print(f"  - {e}")
        print("請確保以下檔案存在於當前目錄:")
        print(f"  1. {transform_file}")
        print(f"  2. {tcp_offset_file}")
        print(f"  3. {gripper_offset_file}")
    except Exception as e:
        print(f"\n錯誤: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n程式結束")


if __name__ == "__main__":
    main()