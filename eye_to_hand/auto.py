import numpy as np
import cv2
import json
import pyrealsense2 as rs
import time
from pymycobot.elephantrobot import ElephantRobot, JogMode
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RealSenseCamera:
    """
    負責管理 RealSense 相機的啟動、停止和內參獲取。
    """
    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.get_intrinsics()

    def get_intrinsics(self):
        """
        啟動 pipeline 並獲取相機內參。
        """
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        
        # 啟動 pipeline 以獲取內參
        profile = self.pipeline.start(self.config)
        
        # 獲取顏色流的內參
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        self.camera_matrix = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ])
        
        # RealSense 相機的畸變係數可以直接從內參物件中獲取
        self.dist_coeffs = np.array(intr.coeffs)
        
        print(f"相機內參自動獲取成功:")
        print(f"Camera Matrix:\n{self.camera_matrix}")
        print(f"Distortion Coefficients:\n{self.dist_coeffs}")
        
        # 停止 pipeline，主程式需要重新啟動
        self.pipeline.stop()
        
    def start_stream(self):
        """
        啟動相機串流。
        """
        self.pipeline.start(self.config)
        
    def stop_stream(self):
        """
        停止相機串流。
        """
        self.pipeline.stop()
        
    def wait_for_frames(self):
        """
        等待並獲取一組 RealSense 框架。
        """
        return self.pipeline.wait_for_frames()


class CameraToRobotTransform:
    """
    處理相機與機械臂之間的坐標轉換。
    """
    def __init__(self, calibration_file=None, camera_matrix=None, dist_coeffs=None):
        self.transformation_matrix = None
        self.calibration_data = None
        self.camera_points = []
        self.robot_points = []
        self.rmse_error = None
        
        # 相機參數
        if camera_matrix is None or dist_coeffs is None:
            raise ValueError("必須提供相機內參和畸變係數")
            
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        # ArUco 設定
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_length = 0.04  # 4cm
        
        if calibration_file:
            self.load_calibration_data(calibration_file)
            self.calculate_transformation()

    def load_calibration_data(self, filename):
        """
        載入標定數據。這個方法能處理包含多個記錄的 JSON 檔案。
        """
        with open(filename, 'r', encoding='utf-8') as f:
            self.calibration_data = json.load(f)
        
        print(f"載入了 {len(self.calibration_data)} 個標定點")
        
        for record in self.calibration_data:
            if record.get('aruco_markers'):
                aruco_pos = record['aruco_markers'][0]['translation_mm']
                camera_point = np.array(aruco_pos)
                
                robot_pos = record['robot_coords'][:3]
                robot_point = np.array(robot_pos)
                
                self.camera_points.append(camera_point)
                self.robot_points.append(robot_point)
        
        self.camera_points = np.array(self.camera_points)
        self.robot_points = np.array(self.robot_points)
        
        print(f"提取了 {len(self.camera_points)} 個有效的對應點")
    
    def calculate_transformation(self):
        """計算轉換矩陣"""
        if len(self.camera_points) < 4:
            raise ValueError("至少需要4個對應點來計算轉換矩陣")
        
        n_points = len(self.camera_points)
        X = np.column_stack([self.camera_points, np.ones(n_points)])
        
        reg_x = LinearRegression(fit_intercept=False)
        reg_y = LinearRegression(fit_intercept=False)
        reg_z = LinearRegression(fit_intercept=False)
        
        reg_x.fit(X, self.robot_points[:, 0])
        reg_y.fit(X, self.robot_points[:, 1])
        reg_z.fit(X, self.robot_points[:, 2])
        
        self.transformation_matrix = np.eye(4)
        self.transformation_matrix[0, :] = reg_x.coef_
        self.transformation_matrix[1, :] = reg_y.coef_
        self.transformation_matrix[2, :] = reg_z.coef_
        
        self.calculate_rmse()
        
        print("轉換矩陣計算完成！")
        print(f"轉換矩陣:\n{self.transformation_matrix}")
        print(f"RMSE 誤差: {self.rmse_error:.2f} mm")
    
    def calculate_rmse(self):
        """計算均方根誤差"""
        if self.transformation_matrix is None:
            return
        
        camera_homogeneous = np.column_stack([self.camera_points, np.ones(len(self.camera_points))])
        predicted_robot = (self.transformation_matrix @ camera_homogeneous.T).T[:, :3]
        
        errors = np.sqrt(np.sum((predicted_robot - self.robot_points)**2, axis=1))
        self.rmse_error = np.sqrt(np.mean(errors**2))
        
        return self.rmse_error
    
    def camera_to_robot(self, camera_coords):
        """將相機坐標轉換為機械臂坐標"""
        if self.transformation_matrix is None:
            raise ValueError("請先計算轉換矩陣")
        
        if len(camera_coords) == 3:
            camera_homogeneous = np.append(camera_coords, 1)
        else:
            camera_homogeneous = camera_coords
        
        robot_homogeneous = self.transformation_matrix @ camera_homogeneous
        
        return robot_homogeneous[:3]
    
    def visualize_calibration(self):
        """視覺化校準結果"""
        if not self.camera_points.any() or not self.robot_points.any():
            return

        fig = plt.figure(figsize=(15, 5))
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.scatter(self.camera_points[:, 0], self.camera_points[:, 1], self.camera_points[:, 2],  
                    c='blue', marker='o', s=50, label='Camera Coordinates')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.set_title('Camera Coordinate System')
        ax1.legend()

        ax2 = fig.add_subplot(132, projection='3d')
        ax2.scatter(self.robot_points[:, 0], self.robot_points[:, 1], self.robot_points[:, 2],  
                    c='red', marker='s', s=50, label='Robot Coordinates')
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.set_zlabel('Z (mm)')
        ax2.set_title('Robot Coordinate System')
        ax2.legend()

        if self.transformation_matrix is not None:
            camera_homogeneous = np.column_stack([self.camera_points, np.ones(len(self.camera_points))])
            predicted_robot = (self.transformation_matrix @ camera_homogeneous.T).T[:, :3]

            ax3 = fig.add_subplot(133, projection='3d')
            ax3.scatter(self.robot_points[:, 0], self.robot_points[:, 1], self.robot_points[:, 2],  
                        c='red', marker='s', s=50, label='Actual Robot Coordinates', alpha=0.7)
            ax3.scatter(predicted_robot[:, 0], predicted_robot[:, 1], predicted_robot[:, 2],  
                        c='green', marker='^', s=50, label='Predicted Robot Coordinates', alpha=0.7)
            ax3.set_xlabel('X (mm)')
            ax3.set_ylabel('Y (mm)')
            ax3.set_zlabel('Z (mm)')
            ax3.set_title(f'Transformation Comparison (RMSE: {self.rmse_error:.2f} mm)')
            ax3.legend()

        plt.tight_layout()
        plt.show()

    def print_calibration_stats(self, predicted_robot=None):
        """印出標定統計資訊"""
        if not self.camera_points.any() or not self.robot_points.any():
            return
        
        print("\n=== 標定統計資訊 ===")
        print(f"標定點數: {len(self.camera_points)}")
        print(f"RMSE 誤差: {self.rmse_error:.2f} mm")
        
        print("\n相機坐標範圍:")
        print(f"  X: {self.camera_points[:, 0].min():.1f} ~ {self.camera_points[:, 0].max():.1f} mm")
        print(f"  Y: {self.camera_points[:, 1].min():.1f} ~ {self.camera_points[:, 1].max():.1f} mm")
        print(f"  Z: {self.camera_points[:, 2].min():.1f} ~ {self.camera_points[:, 2].max():.1f} mm")
        
        print("\n機械臂坐標範圍:")
        print(f"  X: {self.robot_points[:, 0].min():.1f} ~ {self.robot_points[:, 0].max():.1f} mm")
        print(f"  Y: {self.robot_points[:, 1].min():.1f} ~ {self.robot_points[:, 1].max():.1f} mm")
        print(f"  Z: {self.robot_points[:, 2].min():.1f} ~ {self.robot_points[:, 2].max():.1f} mm")
        
        if self.transformation_matrix is not None:
            camera_homogeneous = np.column_stack([self.camera_points, np.ones(len(self.camera_points))])
            predicted_robot = (self.transformation_matrix @ camera_homogeneous.T).T[:, :3]
            errors = np.sqrt(np.sum((predicted_robot - self.robot_points)**2, axis=1))
            
            print(f"\n個別點誤差:")
            for i, error in enumerate(errors):
                print(f"  點 {i+1}: {error:.2f} mm")
    
    def save_transformation_matrix(self, filename):
        """儲存轉換矩陣"""
        if self.transformation_matrix is None:
            print("尚未計算轉換矩陣")
            return
        
        data = {
            'transformation_matrix': self.transformation_matrix.tolist(),
            'rmse_error': float(self.rmse_error),
            'calibration_points': len(self.camera_points),
            'camera_matrix': self.camera_matrix.tolist(),
            'marker_length': self.marker_length
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"轉換矩陣已儲存至: {filename}")
    
    def load_transformation_matrix(self, filename):
        """載入轉換矩陣"""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.transformation_matrix = np.array(data['transformation_matrix'])
        self.rmse_error = data['rmse_error']
        
        print(f"轉換矩陣已載入，RMSE: {self.rmse_error:.2f} mm")


class AutomatedCalibration:
    """
    自動化機械臂移動和數據記錄，用於標定。
    """
    def __init__(self, robot_ip="192.168.50.123", robot_port=5001, coords_speed=2000, cam_manager=None):
        self.robot = ElephantRobot(robot_ip, robot_port)
        if cam_manager is None:
            raise ValueError("必須提供 RealSenseCamera 實例")
        self.cam_manager = cam_manager
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.04  # 4cm
        self.coords_speed = coords_speed

    def _wait_for_robot_to_reach_target(self, target_pose):
        """
        等待機械臂到達目標位置，並在等待期間持續更新相機畫面。
        """
        timeout = 60 # 60秒後超時
        start_time = time.time()
        
        while self.robot.is_in_position(target_pose, JogMode.JOG_TELEOP) != 1 and time.time() - start_time < timeout:
            frames = self.cam_manager.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                cv2.putText(color_image, "Moving to target...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.imshow('Calibration', color_image)
                cv2.waitKey(1)
            
            print(f"\r正在等待機械臂到達目標點... 距離超時還有 {int(timeout - (time.time() - start_time))} 秒", end="", flush=True)
            time.sleep(1)
        
        if self.robot.is_in_position(target_pose, JogMode.JOG_TELEOP) == 1:
            print("\n機械臂已到達目標位置。")
            return True
        else:
            print("\n警告: 機械臂在指定時間內未能到達目標位置。")
            return False

    def run_calibration_sequence(self, target_coords_file, output_file):
        """
        從 JSON 檔案讀取目標坐標，自動移動機械臂並記錄數據。
        """
        try:
            with open(target_coords_file, 'r', encoding='utf-8') as f:
                target_points_data = json.load(f)
            
            target_points = [point['robot_coords'] for point in target_points_data if 'robot_coords' in point]

            print(f"讀取了 {len(target_points)} 個目標機械臂坐標點。")

            self.robot.start_client()
            self.cam_manager.start_stream()
            time.sleep(2)

            collected_data = []

            for i, target_point in enumerate(target_points):
                print(f"\n--- 準備移動至目標點 {i+1}/{len(target_points)} ---")
                
                target_coords_with_pose = target_point
                
                print(f"移動機械臂到: ({target_coords_with_pose[0]:.1f}, {target_coords_with_pose[1]:.1f}, {target_coords_with_pose[2]:.1f}) mm")
                self.robot.write_coords(target_coords_with_pose, self.coords_speed)
                
                if not self._wait_for_robot_to_reach_target(target_coords_with_pose):
                    continue

                time.sleep(2) # 等待穩定

                frames = self.cam_manager.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    print("無法獲取相機影像，跳過此點。")
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                corners, ids, rejected = cv2.aruco.detectMarkers(
                    gray, self.aruco_dict, parameters=self.aruco_params
                )
                
                record = {
                    "record_id": i + 1,
                    "timestamp": time.strftime("%Y%m%d_%H%M%S"),
                    "robot_coords": self.robot.get_coords(),
                    "aruco_markers": []
                }

                if ids is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.cam_manager.camera_matrix, self.cam_manager.dist_coeffs
                    )

                    for j in range(len(ids)):
                        camera_pos = tvecs[j][0] * 1000
                        marker_info = {
                            "id": int(ids[j][0]),
                            "translation_mm": camera_pos.tolist(),
                            "rotation_vector": rvecs[j][0].tolist(),
                            "corners": corners[j].tolist()
                        }
                        record["aruco_markers"].append(marker_info)
                        print(f"  > 檢測到 ArUco ID {ids[j][0]}，相機坐標: ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f}) mm")
                else:
                    print("  > 未檢測到 ArUco 標記，跳過此點。")
                
                collected_data.append(record)

        except Exception as e:
            print(f"處理過程中發生錯誤: {e}")
            
        finally:
            self.cam_manager.stop_stream()
            self.robot.stop_client()
            cv2.destroyAllWindows()
            
            if collected_data:
                with open(output_file, 'w', encoding='utf-8') as f:
                    json.dump(collected_data, f, indent=2, ensure_ascii=False)
                print(f"\n--- 標定數據收集完成！數據已儲存至 {output_file} ---")
            else:
                print("\n--- 未收集到任何有效的標定數據 ---")


def run_live_test(cam_manager, transformer):
    """
    執行即時測試，讀取相機影像，並將 ArUco 標記座標轉換為機械臂座標。
    """
    print("\n\n=== 進入即時測試模式 ===")
    print("請將 ArUco 標記放在相機視野中。按下 'q' 鍵退出。")
    
    # 初始化機械臂連線
    robot = ElephantRobot("192.168.1.159", 5001)
    robot.start_client()

    try:
        cam_manager.start_stream()
        
        # ArUco 設定
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        marker_length = 0.04 # 4cm

        while True:
            # 獲取機械臂當前位置
            current_robot_coords = robot.get_coords()

            frames = cam_manager.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=aruco_params
            )
            
            y_offset = 0
            
            # 顯示當前機械臂位置
            if current_robot_coords:
                cv2.putText(color_image, f"Current Robot Coords:", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(color_image, f"  X: {current_robot_coords[0]:.1f}, Y: {current_robot_coords[1]:.1f}, Z: {current_robot_coords[2]:.1f} mm", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                y_offset = 70

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_length, cam_manager.camera_matrix, cam_manager.dist_coeffs
                )

                for i in range(len(ids)):
                    # 繪製 ArUco 標記
                    cv2.aruco.drawDetectedMarkers(color_image, corners)
                    cv2.drawFrameAxes(color_image, cam_manager.camera_matrix, cam_manager.dist_coeffs, rvecs[i], tvecs[i], marker_length)

                    # 取得相機座標 (毫米)
                    camera_coords = tvecs[i][0] * 1000
                    
                    try:
                        # 轉換為機械臂座標 (毫米)
                        robot_coords_predicted = transformer.camera_to_robot(camera_coords)
                        
                        # 顯示文字資訊
                        cv2.putText(color_image, f"ArUco ID: {ids[i][0]}", (10, y_offset + i * 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        cv2.putText(color_image, f"Camera Coords: ({camera_coords[0]:.1f}, {camera_coords[1]:.1f}, {camera_coords[2]:.1f}) mm", (10, y_offset + 20 + i * 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                        cv2.putText(color_image, f"Predicted Robot Coords:", (10, y_offset + 40 + i * 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(color_image, f"  X: {robot_coords_predicted[0]:.1f}, Y: {robot_coords_predicted[1]:.1f}, Z: {robot_coords_predicted[2]:.1f} mm", (10, y_offset + 60 + i * 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                    except ValueError as e:
                        cv2.putText(color_image, f"Error: {e}", (10, y_offset + 80 + i * 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(color_image, "No ArUco marker detected.", (10, y_offset + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow('Live Test - Camera to Robot Transformation', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"即時測試過程中發生錯誤: {e}")
    finally:
        cam_manager.stop_stream()
        robot.stop_client()
        cv2.destroyAllWindows()


def main():
    """主程式"""
    print("=== 相機到機械臂坐標轉換自動化標定系統 ===")

    target_coords_path = r"C:\Users\user\Downloads\Hand-Eye-Calibration-main\eye_to_hand\aruco_records\complete_record_20260122_133027.json"
    output_calibration_path = r"automated_calibration_data.json"
    final_transform_path = r"camera_to_robot_transform.json"

    # 步驟 1: 初始化 RealSense 相機管理器並自動獲取內參
    print("\n--- 正在初始化 RealSense 相機並自動獲取內參 ---")
    try:
        cam_manager = RealSenseCamera()
    except Exception as e:
        print(f"初始化 RealSense 相機失敗: {e}")
        return

    print("\n--- 請選擇操作模式 ---")
    print("1. 執行完整的自動標定流程 (移動機械臂並收集數據)")
    print("2. 執行即時測試 (載入已有的轉換矩陣並即時顯示)")
    choice = input("請輸入您的選擇 (1 或 2): ")

    if choice == '1':
        print("\n=== 選擇: 執行自動標定流程 ===")
        try:
            with open(target_coords_path, 'r') as f:
                target_data = json.load(f)
                if not isinstance(target_data, list):
                    raise ValueError("目標坐標檔案格式不正確，應為一個包含多個記錄的列表。")
        except FileNotFoundError:
            print(f"錯誤: 找不到目標坐標檔案 {target_coords_path}")
            print("請確保該檔案存在並包含一系列機械臂坐標點。")
            return
        except json.JSONDecodeError:
            print(f"錯誤: 目標坐標檔案 {target_coords_path} 的 JSON 格式無效。")
            return
        except ValueError as e:
            print(f"錯誤: {e}")
            return
        
        # 步驟 2: 執行自動化標定流程
        auto_calibrator = AutomatedCalibration(cam_manager=cam_manager)
        auto_calibrator.run_calibration_sequence(target_coords_path, output_calibration_path)

        # 步驟 3: 使用自動獲取的內參計算轉換矩陣
        print("\n\n=== 計算轉換矩陣並分析結果 ===")
        try:
            transformer = CameraToRobotTransform(
                calibration_file=output_calibration_path,
                camera_matrix=cam_manager.camera_matrix,
                dist_coeffs=cam_manager.dist_coeffs
            )
            transformer.print_calibration_stats()
            transformer.visualize_calibration()
            transformer.save_transformation_matrix(final_transform_path)
        except FileNotFoundError:
            print(f"錯誤: 找不到自動化標定生成的檔案 {output_calibration_path}。")
        except Exception as e:
            print(f"轉換矩陣計算失敗: {e}")

    elif choice == '2':
        print("\n=== 選擇: 執行即時測試 ===")
        # 步驟 2: 載入已有的轉換矩陣
        try:
            transformer = CameraToRobotTransform(
                camera_matrix=cam_manager.camera_matrix,
                dist_coeffs=cam_manager.dist_coeffs
            )
            transformer.load_transformation_matrix(final_transform_path)
            
            # 步驟 3: 執行即時測試
            run_live_test(cam_manager, transformer)
            
        except FileNotFoundError:
            print(f"錯誤: 找不到轉換矩陣檔案 {final_transform_path}。")
            print("請先執行自動標定流程 (選項 1) 來生成此檔案。")
        except Exception as e:
            print(f"即時測試準備階段失敗: {e}")
    else:
        print("無效的選擇。程式結束。")

    print("程式結束")


if __name__ == "__main__":
    main()