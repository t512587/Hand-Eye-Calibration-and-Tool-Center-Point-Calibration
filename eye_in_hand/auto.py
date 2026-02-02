import pyrealsense2 as rs 
import numpy as np
import cv2
import datetime
import json
import os
import time
import math
from pymycobot.elephantrobot import ElephantRobot, JogMode
from scipy.spatial.transform import Rotation as R

# === 棋盤格設定 ===
CHESSBOARD_SIZE = (9, 6)  # 內角點數量 (列, 行)
SQUARE_SIZE = 0.025  # 每個方格的實際大小，單位: 公尺 (25mm)

# === 生成棋盤格 3D 物件點 ===
def create_chessboard_points():
    """生成棋盤格的 3D 世界座標點"""
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE
    return objp

# === 工具函數 ===
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
    """將旋轉矩陣和轉換向量轉換為齊次轉換矩陣"""
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

def perform_handeye_calibration(raw_data):
    """
    執行手眼標定計算 (使用 TSAI 方法)
    """
    print("\n⚙️ 開始手眼標定計算 (TSAI 方法)...")
    
    # === 轉換數據格式 & 單位 ===
    data = []
    for d in raw_data:
        aruco_tvec = [x / 1000 for x in d["aruco_tvec"]]  # mm to m
        robot_pose = d["robot_pose_after_move"]
        robot_pose[:3] = [x / 1000 for x in robot_pose[:3]]  # mm to m
        data.append({
            "aruco_tvec": aruco_tvec,
            "aruco_rvec": d["aruco_rvec"],
            "robot_pose_at_detect": robot_pose
        })
    print("✅ 資料單位轉換完成 (mm -> m)")

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
    print("✅ 旋轉與平移陣列建立完成")

    # === 執行 TSAI 標定 ===
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    H_cam2gripper = to_homogeneous(R_cam2gripper, t_cam2gripper)

    # === 驗證結果 ===
    all_positions = []
    for i in range(len(data)):
        H_gripper2base = to_homogeneous(R_gripper2base[i], t_gripper2base[i])
        H_target2cam = to_homogeneous(R_target2cam[i], t_target2cam[i])
        H_target2base = H_gripper2base @ H_cam2gripper @ H_target2cam
        pos = (H_target2base @ np.array([0, 0, 0, 1]).reshape((4, 1))).ravel()[:3]
        all_positions.append(pos)

    std_dev = np.std(all_positions, axis=0)
    euclidean_std = np.linalg.norm(std_dev) * 1000

    print(f"✅ 標定完成！精度: {euclidean_std:.2f} mm")
    print(f"平移向量 (mm): {t_cam2gripper.ravel() * 1000}")

    # 回傳標定結果
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    calibration_result = {
        "R_cam2gripper": R_cam2gripper.tolist(),
        "t_cam2gripper": t_cam2gripper.ravel().tolist(),
        "H_cam2gripper": H_cam2gripper.tolist(),
        "H_cam2gripper_mm": None,  # 將在後面填入
        "method": "TSAI",
        "timestamp": timestamp,
        "unit": "meters",
        "validation": {
            "euclidean_std_mm": euclidean_std
        }
    }
    
    # 轉換為 mm 單位版本（供即時測試使用）
    H_mm = H_cam2gripper.copy()
    H_mm[:3, 3] *= 1000
    calibration_result["H_cam2gripper_mm"] = H_mm.tolist()
    
    return calibration_result


def detect_chessboard(gray, camera_matrix, dist_coeffs, objp):
    """檢測棋盤格並計算姿態"""
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    ret, corners = cv2.findChessboardCorners(
        gray, CHESSBOARD_SIZE,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    )
    
    if ret:
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        success, rvec, tvec = cv2.solvePnP(
            objp, corners_refined, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if success:
            return True, corners_refined, rvec, tvec
    
    return False, None, None, None


# ============================================================
# 即時測試系統
# ============================================================

class RealTimeTest:
    def __init__(self, H_cam2gripper_mm, robot, pipeline, camera_matrix, dist_coeffs):
        """
        初始化即時測試系統
        
        Args:
            H_cam2gripper_mm: 4x4 相機到夾爪變換矩陣 (mm)
            robot: ElephantRobot 實例
            pipeline: RealSense pipeline
            camera_matrix: 相機內參
            dist_coeffs: 畸變係數
        """
        self.T_camera_gripper = np.array(H_cam2gripper_mm)
        self.robot = robot
        self.pipeline = pipeline
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        self.objp = create_chessboard_points()
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        self.current_transform = None
    
    def invert_transform(self, T):
        """反轉4x4齊次變換矩陣"""
        R_mat = T[:3, :3]
        t = T[:3, 3]
        R_inv = R_mat.T
        t_inv = -R_inv @ t
        T_inv = np.eye(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv
        return T_inv
    
    def transform_point(self, T, P):
        """用4x4變換矩陣轉換3D點"""
        P_h = np.ones(4)
        P_h[:3] = P
        P_transformed = T @ P_h
        return P_transformed[:3]
    
    def camera_to_base_transform(self, camera_point, gripper_pose):
        """將相機座標轉換為基座座標"""
        x, y, z, rx, ry, rz = gripper_pose
        
        translation = np.array([x, y, z])
        rotation = R.from_euler('xyz', [rx, ry, rz], degrees=True)
        rotation_matrix = rotation.as_matrix()
        
        T_base_gripper = np.eye(4)
        T_base_gripper[:3, :3] = rotation_matrix
        T_base_gripper[:3, 3] = translation
        
        T_gripper_camera = self.invert_transform(self.T_camera_gripper)
        P_base = self.transform_point(T_base_gripper @ T_gripper_camera, camera_point)
        
        return P_base
    
    def detect_chessboard(self, color_image):
        """檢測棋盤格並計算其3D位置"""
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(
            gray, CHESSBOARD_SIZE,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )
        
        if ret:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            
            success, rvec, tvec = cv2.solvePnP(
                self.objp, corners_refined, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            
            if success:
                camera_pos = tvec.flatten() * 1000
                return {
                    'detected': True,
                    'camera_position': camera_pos,
                    'corners': corners_refined,
                    'rvec': rvec,
                    'tvec': tvec
                }
        
        return {'detected': False}
    
    def run(self):
        """運行即時測試"""
        print("\n" + "=" * 60)
        print("即時測試模式")
        print("=" * 60)
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
                
                if detection_result['detected']:
                    # 繪製棋盤格
                    cv2.drawChessboardCorners(
                        color_image, CHESSBOARD_SIZE, 
                        detection_result['corners'], True
                    )
                    cv2.drawFrameAxes(
                        color_image, self.camera_matrix, self.dist_coeffs,
                        detection_result['rvec'], detection_result['tvec'],
                        SQUARE_SIZE * 3
                    )
                    
                    # 計算座標轉換
                    try:
                        gripper_pose = self.robot.get_coords()
                        camera_pos = detection_result['camera_position']
                        base_pos = self.camera_to_base_transform(camera_pos, gripper_pose)
                        
                        self.current_transform = {
                            'camera_position': camera_pos,
                            'base_position': base_pos
                        }
                        
                        # 顯示座標
                        cv2.putText(color_image, f"Cam: ({camera_pos[0]:.0f}, {camera_pos[1]:.0f}, {camera_pos[2]:.0f}) mm",
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        cv2.putText(color_image, f"Base: ({base_pos[0]:.0f}, {base_pos[1]:.0f}, {base_pos[2]:.0f}) mm",
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        cv2.putText(color_image, "Chessboard DETECTED", (10, 90),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    except Exception as e:
                        cv2.putText(color_image, f"Error: {e}", (10, 90),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                else:
                    cv2.putText(color_image, "Chessboard NOT found", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    self.current_transform = None
                
                # 顯示按鍵說明
                cv2.putText(color_image, "C: Print coords  Q: Quit", (10, color_image.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow("Real-Time Test", color_image)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == ord('Q'):
                    break
                    
                elif key == ord('c') or key == ord('C'):
                    if self.current_transform:
                        cam = self.current_transform['camera_position']
                        base = self.current_transform['base_position']
                        print(f"\n相機座標: ({cam[0]:.1f}, {cam[1]:.1f}, {cam[2]:.1f}) mm")
                        print(f"基座座標: ({base[0]:.1f}, {base[1]:.1f}, {base[2]:.1f}) mm")
                    else:
                        print("未檢測到棋盤格")
        
        except KeyboardInterrupt:
            print("\n中斷...")


# === 主程式 ===
def main():
    # === 初始化連線 ===
    elephant_client = ElephantRobot("192.168.50.123", 5001)
    elephant_client.start_client()
    print("ElephantRobot目前座標：", elephant_client.get_coords())

    # === 初始化 RealSense 相機 ===
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    cv2.namedWindow("Hand-Eye Calibration (Chessboard)", cv2.WINDOW_AUTOSIZE)

    # === 取得內參 ===
    profile = pipeline.get_active_profile()
    video_stream_profile = profile.get_stream(rs.stream.color)
    intr = video_stream_profile.as_video_stream_profile().get_intrinsics()
    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ])
    dist_coeffs = np.array(intr.coeffs[:5]).reshape(5, 1) if len(intr.coeffs) >= 5 else np.zeros((5, 1))

    print(f"\n=== 相機內參 ===")
    print(f"焦距: fx={intr.fx:.3f}, fy={intr.fy:.3f}")
    print(f"主點: cx={intr.ppx:.3f}, cy={intr.ppy:.3f}")

    # === 棋盤格 3D 點 ===
    objp = create_chessboard_points()

    # === 自動讀取 JSON 開始執行 ===
    json_path = r"C:\Users\user\Downloads\robottest-main\c2h_trans\eye_on_hand\handeye_records\handeye_chessboard_20260119_132556.json"
    
    calibration_result = None
    
    try:
        if not os.path.exists(json_path):
            print(f"JSON file not found: {json_path}")
            print("Please check the path!")
            return
            
        with open(json_path, 'r', encoding='utf-8') as f:
            auto_data = json.load(f)
        
        print(f"Loaded {len(auto_data)} calibration points")
        
        # === 移動到各標定點並記錄 ===
        for i, entry in enumerate(auto_data):
            target_pose = entry["robot_pose_at_detect"]
            print(f"\nMoving to point {i+1}: {target_pose}")
            elephant_client.write_coords(target_pose, 1000)
            
            # 等待到達期間持續更新畫面
            while elephant_client.is_in_position(target_pose, JogMode.JOG_TELEOP) != 1:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                    
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                found, corners, rvec, tvec = detect_chessboard(gray, camera_matrix, dist_coeffs, objp)
                
                if found:
                    cv2.drawChessboardCorners(color_image, CHESSBOARD_SIZE, corners, True)
                    cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec, tvec, SQUARE_SIZE * 3)
                    cv2.putText(color_image, "Chessboard DETECTED", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(color_image, "Chessboard NOT found", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.putText(color_image, f"Moving to point {i+1}/{len(auto_data)}...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.imshow("Hand-Eye Calibration (Chessboard)", color_image)
                cv2.waitKey(1)
                time.sleep(0.05)
            
            # 已到達，記錄實際姿態
            actual_pose = elephant_client.get_coords()
            entry["robot_pose_after_move"] = actual_pose
            print(f"Reached and recorded: {actual_pose}")
        
        print(f"\nAll {len(auto_data)} points completed!")
        
        # === 執行手眼標定計算 ===
        calibration_result = perform_handeye_calibration(auto_data)
        
        if calibration_result:
            # === 儲存標定結果 ===
            save_dir = "calibration_results"
            os.makedirs(save_dir, exist_ok=True)
            result_path = os.path.join(save_dir, f"cam2gripper_{calibration_result['timestamp']}.json")
            
            with open(result_path, 'w', encoding='utf-8') as f:
                json.dump(calibration_result, f, indent=4)
            print(f"\n💾 標定結果已儲存: {result_path}")
            
            # === 顯示標定完成訊息 ===
            print("\n" + "=" * 60)
            print("標定完成！")
            print(f"精度: {calibration_result['validation']['euclidean_std_mm']:.2f} mm")
            print("=" * 60)
            
            # === 進入即時測試模式 ===
            cv2.destroyAllWindows()
            
            realtime_test = RealTimeTest(
                H_cam2gripper_mm=calibration_result["H_cam2gripper_mm"],
                robot=elephant_client,
                pipeline=pipeline,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs
            )
            
            realtime_test.run()
        else:
            print("\n❌ 標定失敗！")
            
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\n程式結束")


if __name__ == "__main__":
    main()