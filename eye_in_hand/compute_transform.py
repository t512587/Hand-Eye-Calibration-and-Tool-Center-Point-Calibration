import numpy as np
import cv2
import json
import os
import math
import pyrealsense2 as rs
from pymycobot.elephantrobot import ElephantRobot
from scipy.spatial.transform import Rotation as R
import time
from collections import deque

# === 棋盤格設定 ===
CHESSBOARD_SIZE = (9, 6)  # 內角點數量 (列, 行)
SQUARE_SIZE = 0.025  # 每個方格的實際大小，單位: 公尺 (25mm)

# === 採集數據 JSON 檔案路徑 ===
CALIBRATION_DATA_JSON_PATH = r"C:\Users\user\Downloads\Hand-Eye-Calibration-main\eye_in_hand\handeye_records\handeye_chessboard_20260204_185505.json"


# ============================================================
# 手眼標定計算相關函數
# ============================================================

def euler_to_rotation_matrix(roll, pitch, yaw, seq="xyz"):
    """將歐拉角轉換為旋轉矩陣"""
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, math.cos(roll), -math.sin(roll)],
                   [0, math.sin(roll), math.cos(roll)]])
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                   [0, 1, 0],
                   [-math.sin(pitch), 0, math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0],
                   [0, 0, 1]])

    if seq == "xyz":
        return Rz @ Ry @ Rx
    elif seq == "zyx":
        return Rx @ Ry @ Rz
    else:
        raise ValueError("不支援的旋轉順序")


def to_homogeneous(R, t):
    """將旋轉矩陣和平移向量轉換為齊次矩陣"""
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3:] = t
    return H


def is_rotation_matrix(R):
    """檢查一個矩陣是否為有效的旋轉矩陣"""
    Rt = np.transpose(R)
    should_be_identity = Rt @ R
    I = np.identity(3, dtype=R.dtype)
    return np.linalg.norm(I - should_be_identity) < 1e-6


def calculate_handeye_calibration(json_path):
    """
    從 JSON 檔案讀取採集數據並計算手眼標定 (使用 TSAI 方法)
    
    Args:
        json_path: 採集數據的 JSON 檔案路徑
        
    Returns:
        np.array: 4x4 相機到夾爪的變換矩陣 (單位: m) ✅ 改為米
    """
    print(f"\n{'='*60}")
    print("手眼標定計算 (TSAI 方法)")
    print(f"{'='*60}")
    print(f"數據檔案: {json_path}")
    
    if not os.path.exists(json_path):
        raise FileNotFoundError(f"找不到數據檔案: {json_path}")
    
    # 載入 JSON 數據
    with open(json_path, 'r', encoding='utf-8') as f:
        raw_data = json.load(f)
    
    print(f"✓ 載入 {len(raw_data)} 筆數據")
    
    # === 轉換數據格式 & 單位 ===
    # ✅ 統一：將所有長度單位從毫米 (mm) 轉換為公尺 (m)
    data = []
    for d in raw_data:
        # 支援新舊兩種格式
        if "chessboard_data" in d:
            # 新格式：使用 chessboard_data
            tvec_mm = d["chessboard_data"]["translation_mm"]
            rvec = d["chessboard_data"]["rotation_vector"]
            robot_pose_key = "robot_coords"
        elif "aruco_tvec" in d:
            # 舊格式：ArUco 標記
            tvec_mm = d["aruco_tvec"]
            rvec = d["aruco_rvec"]
            robot_pose_key = "robot_pose_at_detect"
        else:
            print(f"⚠️ 跳過不支援的數據格式: {d.keys()}")
            continue
        
        aruco_tvec = [x / 1000 for x in tvec_mm]  # mm to m
        robot_pose = d[robot_pose_key].copy()
        robot_pose[:3] = [x / 1000 for x in robot_pose[:3]]  # mm to m
        
        data.append({
            "aruco_tvec": aruco_tvec,
            "aruco_rvec": rvec,
            "robot_pose_at_detect": robot_pose
        })
    
    print("✅ 資料單位轉換完成 (mm -> m)。")
    
    # === 建立旋轉和平移陣列 ===
    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []

    for d in data:
        t_g2b = np.array(d["robot_pose_at_detect"][:3]).reshape((3, 1))
        rpy_g2b = d["robot_pose_at_detect"][3:]
        R_g2b = euler_to_rotation_matrix(*rpy_g2b, seq="xyz")
        R_gripper2base.append(R_g2b)
        t_gripper2base.append(t_g2b)

        rvec_t2c = np.array(d["aruco_rvec"]).reshape((3, 1))
        tvec_t2c = np.array(d["aruco_tvec"]).reshape((3, 1))
        R_t2c, _ = cv2.Rodrigues(rvec_t2c)
        R_target2cam.append(R_t2c)
        t_target2cam.append(tvec_t2c)
    
    # === 使用 TSAI 方法計算 ===
    print("⚙️ 執行 TSAI 手眼標定...")
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )
    
    H_cam2gripper = to_homogeneous(R_cam2gripper, t_cam2gripper)
    
    print("✅ 手眼校正完成！")
    
    # === 顯示結果 ===
    print("\n--- 校正結果 ---")
    print("📷➡️🤖 相機到末端 (Cam to Gripper) 齊次轉換矩陣 H_cam2gripper：")
    print(H_cam2gripper)
    print(f"是否為有效的旋轉矩陣: {is_rotation_matrix(R_cam2gripper)}")
    print(f"平移向量 (m): {t_cam2gripper.ravel()}")
    
    # === 驗證結果 ===
    all_positions = []
    for i in range(len(R_gripper2base)):
        H_gripper2base = to_homogeneous(R_gripper2base[i], t_gripper2base[i])
        H_target2cam = to_homogeneous(R_target2cam[i], t_target2cam[i])
        H_target2base = H_gripper2base @ H_cam2gripper @ H_target2cam
        pos = (H_target2base @ np.array([0, 0, 0, 1]).reshape((4, 1))).ravel()[:3]
        all_positions.append(pos)
    
    avg_pos = np.mean(all_positions, axis=0)
    std_dev = np.std(all_positions, axis=0)
    euclidean_std = np.linalg.norm(std_dev)
    
    print(f"\n所有標定點在基座座標的平均位置 (m): {avg_pos}")
    print(f"所有標定點在基座座標的標準差 (m) (越小越好): {std_dev}")
    print(f"歐氏距離標準差 (m): {euclidean_std:.6f}")
    print(f"✓ 標定完成，精度: {euclidean_std * 1000:.2f} mm")
    
    # ✅ 保持米單位，不轉換為毫米
    return H_cam2gripper  # 返回米單位的矩陣


# ============================================================
# 實時座標轉換系統
# ============================================================

class RealTimeCoordinateTransform:
    def __init__(self, camera_to_gripper_matrix, robot_ip="192.168.50.123", robot_port=5001):
        """
        初始化實時座標轉換系統
        
        Args:
            camera_to_gripper_matrix: 相機到夾爪的變換矩陣 (單位: m) ✅
        """
        self.T_camera_gripper = np.array(camera_to_gripper_matrix)  # 單位: 米
        
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot = None
        self.robot_connected = False
        
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1))
        
        self.chessboard_size = CHESSBOARD_SIZE
        self.square_size = SQUARE_SIZE
        self.objp = self._create_chessboard_points()
        
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        self.pipeline = None
        self.current_transform = None
        
        print("實時座標轉換系統初始化完成")
        print(f"⚙️ T_camera_gripper 單位: 米 (m)")
    
    def _create_chessboard_points(self):
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp
    
    def connect_robot(self):
        try:
            self.robot = ElephantRobot(self.robot_ip, self.robot_port)
            self.robot.start_client()
            self.robot_connected = True
            print(f"✓ 機械手臂連接成功: {self.robot_ip}:{self.robot_port}")
            return True
        except Exception as e:
            print(f"⚠️ 機械手臂連接失敗: {e}")
            self.robot_connected = False
            return False
    
    def init_camera(self):
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            profile = self.pipeline.start(config)
            
            color_stream = profile.get_stream(rs.stream.color)
            intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            self.camera_matrix = np.array([
                [intrinsics.fx, 0, intrinsics.ppx],
                [0, intrinsics.fy, intrinsics.ppy],
                [0, 0, 1]
            ])
            
            if len(intrinsics.coeffs) >= 5:
                self.dist_coeffs = np.array(intrinsics.coeffs[:5]).reshape(5, 1)
            
            print(f"\n✓ 相機內參: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
            print("✓ RealSense相機初始化成功")
            return True
            
        except Exception as e:
            print(f"❌ 相機初始化失敗: {e}")
            return False
    
    def invert_transform(self, T):
        """計算變換矩陣的逆"""
        R_mat = T[:3, :3]
        t = T[:3, 3]
        R_inv = R_mat.T
        t_inv = -R_inv @ t
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        return T_inv
    
    def transform_point(self, T, P):
        """使用變換矩陣轉換 3D 點"""
        P_h = np.ones(4)
        P_h[:3] = P
        P_transformed = T @ P_h
        return P_transformed[:3]
    
    def camera_to_base_transform(self, rvec, tvec, gripper_pose):
        """
        將棋盤格從相機座標系轉換到基座座標系
        ✅ 統一使用米 (m) 進行計算
        
        Args:
            rvec: 旋轉向量 (from solvePnP)
            tvec: 平移向量 (from solvePnP), 單位 m
            gripper_pose: [x, y, z, rx, ry, rz], 位置單位 mm
        
        Returns:
            棋盤格在基座座標系的位置 [x, y, z] (m) ✅ 改為返回米
        """
        x, y, z, rx, ry, rz = gripper_pose
        
        # 步驟 1: 建立 T_base_gripper (單位: m)
        translation = np.array([x, y, z]) / 1000.0  # ✅ mm → m
        rotation = R.from_euler('xyz', [rx, ry, rz], degrees=True)
        rotation_matrix = rotation.as_matrix()
        
        T_base_gripper = np.eye(4)
        T_base_gripper[:3, :3] = rotation_matrix
        T_base_gripper[:3, 3] = translation  # 單位: m
        
        # 步驟 2: 計算 T_gripper_camera = inverse(T_camera_gripper)
        T_gripper_camera = self.invert_transform(self.T_camera_gripper)  # 單位: m
        
        # 步驟 3: 建立 T_camera_target (棋盤格在相機座標系, 單位: m)
        R_camera_target, _ = cv2.Rodrigues(rvec)
        t_camera_target = tvec.flatten()  # ✅ 保持米單位，不轉換
        
        T_camera_target = np.eye(4)
        T_camera_target[:3, :3] = R_camera_target
        T_camera_target[:3, 3] = t_camera_target  # 單位: m
        
        # 步驟 4: 完整變換鏈（全部使用米）
        # T_base_target = T_base_gripper @ T_gripper_camera @ T_camera_target
        T_base_target = T_base_gripper @ T_gripper_camera @ T_camera_target
        
        # 步驟 5: 提取棋盤格原點在基座座標系的位置
        target_origin = np.array([0, 0, 0, 1])
        base_position = (T_base_target @ target_origin)[:3]  # 單位: m
        
        return base_position
    
    def detect_chessboard(self, color_image):
        """檢測棋盤格"""
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(
            gray, self.chessboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )
        
        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            
            success, rvec, tvec = cv2.solvePnP(
                self.objp, corners_refined, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if success:
                camera_pos = tvec.flatten()  # ✅ 單位: m
                return {
                    'detected': True,
                    'camera_position': camera_pos,
                    'corners': corners_refined,
                    'rvec': rvec,
                    'tvec': tvec
                }
        
        return {'detected': False}
    
    def draw_results(self, color_image, detection_result):
        """繪製檢測結果"""
        if detection_result['detected']:
            cv2.drawChessboardCorners(
                color_image, self.chessboard_size, 
                detection_result['corners'], True
            )
            
            cv2.drawFrameAxes(
                color_image, self.camera_matrix, self.dist_coeffs,
                detection_result['rvec'], detection_result['tvec'],
                self.square_size * 3
            )
            
            # ✅ 顯示時轉換為毫米
            cam_pos = detection_result['camera_position'] * 1000  # m → mm
            cv2.putText(color_image, f"Cam: ({cam_pos[0]:.0f}, {cam_pos[1]:.0f}, {cam_pos[2]:.0f}) mm",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            
            if self.current_transform:
                base_pos = self.current_transform['base_position'] * 1000  # m → mm
                cv2.putText(color_image, f"Base: ({base_pos[0]:.0f}, {base_pos[1]:.0f}, {base_pos[2]:.0f}) mm",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.putText(color_image, "Chessboard DETECTED", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(color_image, "Chessboard NOT found", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # 顯示按鍵說明
        cv2.putText(color_image, "C: Print coords  Q: Quit", (10, color_image.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def run(self):
        """執行實時座標轉換"""
        if not self.init_camera():
            return
        
        self.connect_robot()
        
        print(f"\n{'='*60}")
        print("=== 實時座標轉換系統啟動 ===")
        print("單位系統: 內部計算使用米 (m), 顯示使用毫米 (mm)")
        print("='*60}")
        print("按 'C' 鍵: 輸出當前座標")
        print("按 'Q' 鍵: 退出")
        print("=" * 60)
        
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                
                detection_result = self.detect_chessboard(color_image)
                
                if self.robot_connected and detection_result['detected']:
                    try:
                        current_gripper_pose = self.robot.get_coords()
                        
                        # 使用完整的 rvec 和 tvec 進行轉換（單位: m）
                        base_pos = self.camera_to_base_transform(
                            detection_result['rvec'],
                            detection_result['tvec'],
                            current_gripper_pose
                        )
                        
                        camera_pos = detection_result['camera_position']  # 單位: m
                        
                        self.current_transform = {
                            'camera_position': camera_pos,  # m
                            'base_position': base_pos  # m
                        }
                    except Exception as e:
                        print(f"⚠️ 座標轉換錯誤: {e}")
                
                self.draw_results(color_image, detection_result)
                
                cv2.imshow('Coordinate Transform', color_image)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    break
                    
                elif key == ord('c') or key == ord('C'):
                    if self.current_transform:
                        # ✅ 顯示時轉換為毫米
                        cam = self.current_transform['camera_position'] * 1000  # m → mm
                        base = self.current_transform['base_position'] * 1000  # m → mm
                        print(f"\n相機座標: ({cam[0]:.1f}, {cam[1]:.1f}, {cam[2]:.1f}) mm")
                        print(f"基座座標: ({base[0]:.1f}, {base[1]:.1f}, {base[2]:.1f}) mm")
                    else:
                        print("⚠️ 未檢測到棋盤格")
        
        except KeyboardInterrupt:
            print("\n⛔ 中斷...")
        
        finally:
            if self.pipeline:
                self.pipeline.stop()
            cv2.destroyAllWindows()
            print("✅ 系統已關閉")


# ============================================================
# 主程式
# ============================================================

def main():
    print("=" * 60)
    print("實時相機到機械手臂座標轉換系統")
    print("單位系統: 統一使用米 (m) 進行計算")
    print("=" * 60)
    
    # === 輸入採集數據 JSON 檔案路徑 ===
    json_path = input(f"\n請輸入數據 JSON 路徑 (Enter 使用預設): ").strip()
    if not json_path:
        json_path = CALIBRATION_DATA_JSON_PATH
    
    # === 計算手眼標定 ===
    try:
        T_camera_gripper = calculate_handeye_calibration(json_path)
        print(f"\n✅ 手眼標定矩陣已計算完成 (單位: m)")
    except Exception as e:
        print(f"\n❌ 標定失敗: {e}")
        return
    
    # === 機器人 IP ===
    robot_ip = input("\n機器人 IP (Enter 使用 192.168.50.123): ").strip()
    if not robot_ip:
        robot_ip = "192.168.50.123"
    
    # === 啟動 ===
    choice = input("\n啟動實時系統？(y/n): ").strip().lower()
    if choice != 'y':
        return
    
    transform_system = RealTimeCoordinateTransform(
        camera_to_gripper_matrix=T_camera_gripper,
        robot_ip=robot_ip,
        robot_port=5001
    )
    
    transform_system.run()


if __name__ == "__main__":
    main()