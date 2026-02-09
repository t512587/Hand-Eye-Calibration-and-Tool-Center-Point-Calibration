import os
import cv2
import numpy as np
from math import *
import json


class ArucoHandEyeCalibration:
    def __init__(self):
        # 相機內參矩陣
        self.K = np.array([[616.798, 0.00000000e+00, 321.753],
                           [0.00000000e+00, 616.904, 247.541],
                           [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float64)
        self.distortion = np.zeros((5, 1), dtype=np.float64)  # 畸變係數
        
        # ArUco 設定
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_length = 0.04  # ArUco 標記邊長 (米)

    def angle2rotation(self, x, y, z):
        """將歐拉角轉換為旋轉矩陣"""
        Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
        Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
        Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R

    def gripper2base(self, x, y, z, tx, ty, tz):
        """計算夾爪到基座的變換矩陣"""
        thetaX = x / 180 * pi
        thetaY = y / 180 * pi
        thetaZ = z / 180 * pi
        R_gripper2base = self.angle2rotation(thetaX, thetaY, thetaZ)
        T_gripper2base = np.array([[tx], [ty], [tz]])
        Matrix_gripper2base = np.column_stack([R_gripper2base, T_gripper2base])
        Matrix_gripper2base = np.vstack((Matrix_gripper2base, np.array([0, 0, 0, 1])))
        R_gripper2base = Matrix_gripper2base[:3, :3]
        T_gripper2base = Matrix_gripper2base[:3, 3].reshape((3, 1))
        return R_gripper2base, T_gripper2base

    def process_from_json(self, json_file, marker_id=0):
        """
        從 JSON 文件處理手眼標定
        Args:
            json_file: JSON 標定記錄文件路徑
            marker_id: ArUco 標記 ID（用於選擇特定標記，如果有多個）
        Returns:
            R_camera2base: 相機到基座的旋轉矩陣
            T_camera2base: 相機到基座的平移向量
            其他驗證所需的矩陣列表
        """
        # 讀取 JSON 數據
        with open(json_file, 'r', encoding='utf-8') as f:
            calibration_data = json.load(f)
        
        R_base2gripper_list = []
        T_base2gripper_list = []
        R_target2camera_list = []
        T_target2camera_list = []

        
        print(f"開始處理 {len(calibration_data)} 筆記錄...")
        
        for idx, record in enumerate(calibration_data):
            try:
                # 檢查是否有 ArUco 標記
                if not record.get('aruco_markers') or len(record['aruco_markers']) == 0:
                    print(f"記錄 {idx + 1}: 沒有 ArUco 標記，跳過")
                    continue
                
                # 獲取機械臂姿態
                robot_coords = record['robot_coords']
                x, y, z = np.array(robot_coords[:3]) / 1000.0
                rx, ry, rz = robot_coords[3:6]
                R_gripper2base, T_gripper2base = self.gripper2base(rx, ry, rz, x, y, z)

                # Eye-to-Hand：需要 base -> gripper
                R_base2gripper = R_gripper2base.T
                T_base2gripper = -R_gripper2base.T @ T_gripper2base



                
                # 從 JSON 中的 ArUco 數據獲取標記姿態
                # 選擇第一個標記（或指定 ID 的標記）
                marker_data = None
                if marker_id is None:
                    # 使用第一個標記
                    marker_data = record['aruco_markers'][0]
                else:
                    # 尋找指定 ID 的標記
                    for marker in record['aruco_markers']:
                        if marker.get('id') == marker_id:
                            marker_data = marker
                            break
                
                if marker_data is None:
                    # 如果沒找到指定 ID，嘗試使用第一個
                    if len(record['aruco_markers']) > 0:
                        marker_data = record['aruco_markers'][0]
                        print(f"記錄 {idx + 1}: 未找到 ID {marker_id}，使用第一個標記 ID {marker_data.get('id', 'unknown')}")
                    else:
                        print(f"記錄 {idx + 1}: 沒有可用的標記，跳過")
                        continue
                
                # 檢查數據格式並提取姿態
                if 'rotation_vector' in marker_data and 'translation_mm' in marker_data:
                    rvec = np.array(marker_data['rotation_vector'], dtype=np.float64).reshape((3, 1))
                    tvec = np.array(marker_data['translation_mm'], dtype=np.float64).reshape((3, 1)) / 1000.0
                else:
                    print(f"記錄 {idx + 1}: 缺少 rotation_vector 或 translation_mm，跳過")
                    continue

                
                # 轉換為旋轉矩陣
                R_target2camera, _ = cv2.Rodrigues(rvec)
                T_target2camera = tvec
                
                # 添加到列表
                R_base2gripper_list.append(R_base2gripper)
                T_base2gripper_list.append(T_base2gripper)

                R_target2camera_list.append(R_target2camera)
                T_target2camera_list.append(T_target2camera)
                
                print(f"成功處理記錄 {idx + 1}/{len(calibration_data)}")
                
            except Exception as e:
                print(f"記錄 {idx + 1} 處理失敗: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        if len(R_target2camera_list) == 0:
            raise ValueError("沒有成功處理任何記錄")
        
        print(f"\n總共成功處理 {len(R_target2camera_list)} 筆數據")
        
        # 執行手眼標定
        print("\n開始執行手眼標定...")
        R_camera2base, T_camera2base = cv2.calibrateHandEye(
            R_base2gripper_list, T_base2gripper_list,
            R_target2camera_list, T_target2camera_list,
            method=cv2.CALIB_HAND_EYE_TSAI)

        
        print("手眼標定完成！")
        
        return (
            R_camera2base,
            T_camera2base,
            R_base2gripper_list,
            T_base2gripper_list,
            R_target2camera_list,
            T_target2camera_list
        )




    def check_result(self,
                    R_camera2base, T_camera2base,
                    R_base2gripper_list, T_base2gripper_list,
                    R_target2camera_list, T_target2camera_list):

        print("\n=== 標定結果驗證 ===")
        print(f"\n相機到基座的旋轉矩陣 R_camera2base:\n{R_camera2base}")
        print(f"\n相機到基座的平移向量 T_camera2base (m):\n{T_camera2base}")

        errors = []
        prev_RT = None

        for i in range(len(R_base2gripper_list)):
            # base -> gripper
            RT_base2gripper = np.column_stack(
                (R_base2gripper_list[i], T_base2gripper_list[i])
            )
            RT_base2gripper = np.vstack((RT_base2gripper, [0, 0, 0, 1]))

            # camera -> base
            RT_camera2base = np.column_stack((R_camera2base, T_camera2base))
            RT_camera2base = np.vstack((RT_camera2base, [0, 0, 0, 1]))

            # target -> camera
            RT_target2camera = np.column_stack(
                (R_target2camera_list[i], T_target2camera_list[i])
            )
            RT_target2camera = np.vstack((RT_target2camera, [0, 0, 0, 1]))

            # target -> gripper (Eye-to-Hand 正確鏈式)
            RT_target2gripper = (
                RT_base2gripper @
                RT_camera2base @
                RT_target2camera
            )

            print(f"\n第 {i+1} 次驗證結果:")
            print(f"標定板到夾爪的變換矩陣:\n{RT_target2gripper}")

            if prev_RT is not None:
                error = np.linalg.norm(RT_target2gripper - prev_RT)
                errors.append(error)
                print(f"與前一次的差異: {error:.6f}")

            prev_RT = RT_target2gripper

        if errors:
            print(f"\n平均誤差: {np.mean(errors):.6f}")
            print(f"最大誤差: {np.max(errors):.6f}")
            print(f"最小誤差: {np.min(errors):.6f}")


    def save_calibration_result(self, R_camera2base, T_camera2base, filename):
        """保存標定結果"""
        # 構建完整的 4x4 變換矩陣
        transformation_matrix = np.column_stack((R_camera2base, T_camera2base))
        transformation_matrix = np.vstack((transformation_matrix, np.array([0, 0, 0, 1])))
        
        result = {
            'rotation_matrix': R_camera2base.tolist(),
            'translation_vector': T_camera2base.tolist(),
            'transformation_matrix': transformation_matrix.tolist(),
            'camera_matrix': self.K.tolist(),
            'distortion': self.distortion.tolist(),
            'marker_length': self.marker_length
        }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        
        print(f"\n標定結果已保存至: {filename}")


if __name__ == "__main__":
    calibrator = ArucoHandEyeCalibration()

    print("=== ArUco 手眼標定系統 (Eye-to-Hand) ===")

    json_file = r"C:\Users\user\Downloads\Hand-Eye-Calibration-main\eye_to_hand\aruco_records\complete_record_20260203_112200.json"
    marker_id = 0

    try:
        print(f"\n讀取標定數據: {json_file}")
        print(f"使用 ArUco 標記 ID: {marker_id}\n")

        # ✅ 一定要先做標定
        (
            R_camera2base,
            T_camera2base,
            R_base2gripper_list,
            T_base2gripper_list,
            R_target2camera_list,
            T_target2camera_list
        ) = calibrator.process_from_json(json_file, marker_id)

        # ✅ 驗證標定結果
        calibrator.check_result(
            R_camera2base, T_camera2base,
            R_base2gripper_list, T_base2gripper_list,
            R_target2camera_list, T_target2camera_list
        )

        # ✅ 保存結果
        calibrator.save_calibration_result(
            R_camera2base,
            T_camera2base,
            'aruco_hand_eye_calibration_result.json'
        )

        print("\n=== 標定完成 ===")
        print("相機 → 基座 (camera → base) 變換已計算完成")

    except Exception as e:
        print(f"\n錯誤: {e}")
        import traceback
        traceback.print_exc()