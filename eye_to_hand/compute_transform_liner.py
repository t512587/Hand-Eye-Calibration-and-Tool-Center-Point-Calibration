import numpy as np
import cv2
import json
import pyrealsense2 as rs
from pymycobot.elephantrobot import ElephantRobot
from scipy.spatial.transform import Rotation as R
from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class CameraToRobotTransform:
    def __init__(self, calibration_file=None):
        self.transformation_matrix = None
        self.calibration_data = None
        self.camera_points = []
        self.robot_points = []
        self.rmse_error = None
        
        # 相機參數
        self.camera_matrix = np.array([[616.798, 0, 321.753],
                                      [0, 616.904, 247.541],
                                      [0, 0, 1]])
        self.dist_coeffs = np.zeros((5, 1))
        
        # ArUco 設定
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_length = 0.04  # 5cm
        
        if calibration_file:
            self.load_calibration_data(calibration_file)
            self.calculate_transformation()
    
    def load_calibration_data(self, filename):
        """載入標定數據"""
        with open(filename, 'r', encoding='utf-8') as f:
            self.calibration_data = json.load(f)
        
        print(f"載入了 {len(self.calibration_data)} 個標定點")
        
        # 提取相機坐標和機械臂坐標
        for record in self.calibration_data:
            if record['aruco_markers']:  # 確保有檢測到標記
                # 取第一個 ArUco 標記的位置 (相機坐標系)
                aruco_pos = record['aruco_markers'][0]['translation_mm']
                camera_point = np.array(aruco_pos)
                
                # 機械臂坐標 (只取 X, Y, Z)
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
        
        # 使用最小二乘法求解仿射變換矩陣
        n_points = len(self.camera_points)
        
        # 為每個軸建立線性回歸模型
        X = np.column_stack([self.camera_points, np.ones(n_points)])
        
        # 分別為 X, Y, Z 軸建立回歸模型
        reg_x = LinearRegression(fit_intercept=False)
        reg_y = LinearRegression(fit_intercept=False)
        reg_z = LinearRegression(fit_intercept=False)
        
        reg_x.fit(X, self.robot_points[:, 0])
        reg_y.fit(X, self.robot_points[:, 1])
        reg_z.fit(X, self.robot_points[:, 2])
        
        # 建立 4x4 轉換矩陣
        self.transformation_matrix = np.eye(4)
        self.transformation_matrix[0, :] = reg_x.coef_
        self.transformation_matrix[1, :] = reg_y.coef_
        self.transformation_matrix[2, :] = reg_z.coef_
        
        # 計算轉換誤差
        self.calculate_rmse()
        
        print("轉換矩陣計算完成！")
        print(f"轉換矩陣:\n{self.transformation_matrix}")
        print(f"RMSE 誤差: {self.rmse_error:.2f} mm")
    
    def calculate_rmse(self):
        """計算均方根誤差"""
        if self.transformation_matrix is None:
            return
        
        # 使用轉換矩陣預測機械臂坐標
        camera_homogeneous = np.column_stack([self.camera_points, np.ones(len(self.camera_points))])
        predicted_robot = (self.transformation_matrix @ camera_homogeneous.T).T[:, :3]
        
        # 計算 RMSE
        errors = np.sqrt(np.sum((predicted_robot - self.robot_points)**2, axis=1))
        self.rmse_error = np.sqrt(np.mean(errors**2))
        
        return self.rmse_error
    
    def camera_to_robot(self, camera_coords):
        """將相機坐標轉換為機械臂坐標"""
        if self.transformation_matrix is None:
            raise ValueError("請先計算轉換矩陣")
        
        # 轉換為齊次坐標
        if len(camera_coords) == 3:
            camera_homogeneous = np.append(camera_coords, 1)
        else:
            camera_homogeneous = camera_coords
        
        # 應用轉換矩陣
        robot_homogeneous = self.transformation_matrix @ camera_homogeneous
        
        return robot_homogeneous[:3]
    
    def visualize_calibration(self):
        """Visualize calibration results"""
        if not self.camera_points.any() or not self.robot_points.any():
            return

        fig = plt.figure(figsize=(15, 5))

        # Camera coordinate system
        ax1 = fig.add_subplot(131, projection='3d')
        ax1.scatter(self.camera_points[:, 0], self.camera_points[:, 1], self.camera_points[:, 2], 
                c='blue', marker='o', s=50, label='Camera Coordinates')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.set_title('Camera Coordinate System')
        ax1.legend()

        # Robot coordinate system
        ax2 = fig.add_subplot(132, projection='3d')
        ax2.scatter(self.robot_points[:, 0], self.robot_points[:, 1], self.robot_points[:, 2], 
                c='red', marker='s', s=50, label='Robot Coordinates')
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.set_zlabel('Z (mm)')
        ax2.set_title('Robot Coordinate System')
        ax2.legend()

        # Transformation comparison
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

    def print_calibration_stats(self):
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
        
        # 個別點的誤差
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


class RealTimeTransform:
    def __init__(self, transformer, robot_ip="192.168.50.123", robot_port=5001):
        self.transformer = transformer
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        
        # 初始化機械臂連線
        self.robot = ElephantRobot(robot_ip, robot_port)
        self.robot.start_client()
        
        # 初始化相機
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        
        print("即時轉換系統已啟動")
        print("操作說明:")
        print("- 按 'c' 鍵獲取當前檢測到的ArUco標記坐標並轉換")
        print("- 按 'm' 鍵移動機械臂到轉換後的坐標")
        print("- 按 't' 鍵啟動自動測試模式")
        print("- 按 'q' 鍵退出")
    
    def run(self):
        """執行即時轉換"""
        current_robot_target = None
        
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
                    gray, self.transformer.aruco_dict, parameters=self.transformer.parameters)
                
                if ids is not None:
                    # 估計姿態
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.transformer.marker_length, 
                        self.transformer.camera_matrix, self.transformer.dist_coeffs)
                    
                    # 繪製檢測結果
                    for i in range(len(ids)):
                        cv2.aruco.drawDetectedMarkers(color_image, corners)
                        cv2.drawFrameAxes(color_image, self.transformer.camera_matrix, 
                                        self.transformer.dist_coeffs, rvecs[i], tvecs[i], 0.03)
                        
                        # 顯示相機坐標
                        camera_pos = tvecs[i][0] * 1000  # 轉換為 mm
                        cv2.putText(color_image, f"ID: {ids[i][0]}", 
                                  (int(corners[i][0][0][0]), int(corners[i][0][0][1])-40), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(color_image, f"Cam: ({camera_pos[0]:.0f}, {camera_pos[1]:.0f}, {camera_pos[2]:.0f})", 
                                  (int(corners[i][0][0][0]), int(corners[i][0][0][1])-25), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                        
                        # 轉換為機械臂坐標
                        robot_pos = self.transformer.camera_to_robot(camera_pos)
                        cv2.putText(color_image, f"Robot: ({robot_pos[0]:.0f}, {robot_pos[1]:.0f}, {robot_pos[2]:.0f})", 
                                  (int(corners[i][0][0][0]), int(corners[i][0][0][1])-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                
                # 顯示當前機械臂位置
                current_robot_pos = self.robot.get_coords()
                cv2.putText(color_image, f"Current Robot: ({current_robot_pos[0]:.0f}, {current_robot_pos[1]:.0f}, {current_robot_pos[2]:.0f})", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                
                if current_robot_target is not None:
                    cv2.putText(color_image, f"Target: ({current_robot_target[0]:.0f}, {current_robot_target[1]:.0f}, {current_robot_target[2]:.0f})", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                cv2.imshow('Real-time Camera to Robot Transform', color_image)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('c') and ids is not None:
                    # 獲取並轉換坐標
                    print("\n=== 坐標轉換 ===")
                    for i in range(len(ids)):
                        camera_pos = tvecs[i][0] * 1000
                        robot_pos = self.transformer.camera_to_robot(camera_pos)
                        current_robot_target = robot_pos
                        
                        print(f"ArUco ID: {ids[i][0]}")
                        print(f"相機坐標: ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f}) mm")
                        print(f"機械臂坐標: ({robot_pos[0]:.1f}, {robot_pos[1]:.1f}, {robot_pos[2]:.1f}) mm")
                        print("-" * 40)
                
                elif key == ord('m') and current_robot_target is not None:
                    # 移動機械臂
                    print(f"\n移動機械臂到目標位置: ({current_robot_target[0]:.1f}, {current_robot_target[1]:.1f}, {current_robot_target[2]:.1f})")
                    
                    # 保持當前的姿態角度
                    current_coords = self.robot.get_coords()
                    target_coords = [current_robot_target[0], current_robot_target[1], current_robot_target[2],
                                   current_coords[3], current_coords[4], current_coords[5]]
                    
                    try:
                        self.robot.send_coords(target_coords, 50)  # 速度 50
                        print("移動指令已發送")
                    except Exception as e:
                        print(f"移動失敗: {e}")
                
                elif key == ord('t'):
                    # 啟動自動測試模式
                    print("\n=== 啟動自動測試模式 ===")
                    self.auto_test_mode()
        
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
    
    def auto_test_mode(self):
        """自動測試模式 - 驗證標定精度"""
        print("\n請將 ArUco 標記固定在一個位置")
        print("機械臂將從不同位置檢測並計算誤差")
        input("準備好後按 Enter 繼續...")
        
        # 記錄當前機械臂位置以便恢復
        original_pos = self.robot.get_coords()
        
        # 測試點列表 (相對於當前位置的偏移)
        test_offsets = [
            [0, 0, 0],      # 當前位置
            [30, 0, 0],     # X+30mm
            [-30, 0, 0],    # X-30mm
            [0, 30, 0],     # Y+30mm
            [0, -30, 0],    # Y-30mm
            [0, 0, 30],     # Z+30mm
            [0, 0, -30],    # Z-30mm
        ]
        
        test_results = []
        
        print("\n開始自動測試...")
        
        for idx, offset in enumerate(test_offsets):
            print(f"\n--- 測試點 {idx + 1}/{len(test_offsets)} ---")
            print(f"偏移: ({offset[0]}, {offset[1]}, {offset[2]}) mm")
            
            # 計算目標位置
            target_pos = [
                original_pos[0] + offset[0],
                original_pos[1] + offset[1],
                original_pos[2] + offset[2],
                original_pos[3], original_pos[4], original_pos[5]
            ]
            
            # 移動到測試位置
            print(f"移動到測試位置...")
            self.robot.send_coords(target_pos, 30)
            time.sleep(3)  # 等待移動完成
            
            # 獲取當前實際位置
            actual_pos = self.robot.get_coords()[:3]
            print(f"實際機械臂位置: ({actual_pos[0]:.1f}, {actual_pos[1]:.1f}, {actual_pos[2]:.1f}) mm")
            
            # 等待穩定
            time.sleep(1)
            
            # 檢測 ArUco 標記並計算位置
            detected_positions = []
            for attempt in range(5):  # 多次檢測求平均
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self.transformer.aruco_dict, parameters=self.transformer.parameters)
                
                if ids is not None:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.transformer.marker_length,
                        self.transformer.camera_matrix, self.transformer.dist_coeffs)
                    
                    camera_pos = tvecs[0][0] * 1000
                    robot_pos = self.transformer.camera_to_robot(camera_pos)
                    detected_positions.append(robot_pos)
                
                time.sleep(0.1)
            
            if detected_positions:
                # 計算平均位置
                avg_detected_pos = np.mean(detected_positions, axis=0)
                
                # 計算誤差
                error = np.linalg.norm(np.array(actual_pos) - avg_detected_pos)
                error_vec = np.array(actual_pos) - avg_detected_pos
                
                result = {
                    'test_point': idx + 1,
                    'offset': offset,
                    'actual_pos': actual_pos,
                    'detected_pos': avg_detected_pos.tolist(),
                    'error': float(error),
                    'error_vector': error_vec.tolist(),
                    'num_samples': len(detected_positions)
                }
                
                test_results.append(result)
                
                print(f"檢測到的位置: ({avg_detected_pos[0]:.1f}, {avg_detected_pos[1]:.1f}, {avg_detected_pos[2]:.1f}) mm")
                print(f"位置誤差: {error:.2f} mm")
                print(f"誤差向量: X={error_vec[0]:.2f}, Y={error_vec[1]:.2f}, Z={error_vec[2]:.2f} mm")
            else:
                print("未檢測到 ArUco 標記!")
        
        # 返回原始位置
        print("\n返回原始位置...")
        self.robot.send_coords(original_pos, 30)
        time.sleep(2)
        
        # 顯示測試總結
        self.print_test_summary(test_results)
        
        # 儲存測試結果
        self.save_test_results(test_results)
    
    def print_test_summary(self, results):
        """印出測試總結"""
        if not results:
            print("\n沒有有效的測試結果")
            return
        
        print("\n" + "=" * 60)
        print("=== 自動測試總結 ===")
        print("=" * 60)
        
        errors = [r['error'] for r in results]
        
        print(f"\n測試點數: {len(results)}")
        print(f"平均誤差: {np.mean(errors):.2f} mm")
        print(f"最大誤差: {np.max(errors):.2f} mm")
        print(f"最小誤差: {np.min(errors):.2f} mm")
        print(f"標準差: {np.std(errors):.2f} mm")
        
        print("\n各測試點詳細結果:")
        print("-" * 60)
        for result in results:
            print(f"\n測試點 {result['test_point']}:")
            print(f"  偏移: {result['offset']}")
            print(f"  實際位置: ({result['actual_pos'][0]:.1f}, {result['actual_pos'][1]:.1f}, {result['actual_pos'][2]:.1f})")
            print(f"  檢測位置: ({result['detected_pos'][0]:.1f}, {result['detected_pos'][1]:.1f}, {result['detected_pos'][2]:.1f})")
            print(f"  誤差: {result['error']:.2f} mm")
            print(f"  誤差分量: X={result['error_vector'][0]:.2f}, Y={result['error_vector'][1]:.2f}, Z={result['error_vector'][2]:.2f}")
        
        # 誤差評估
        print("\n" + "=" * 60)
        print("=== 精度評估 ===")
        avg_error = np.mean(errors)
        max_error = np.max(errors)
        
        if avg_error < 5 and max_error < 8:
            print("✅ 優秀! 標定精度非常好,適合精密操作")
        elif avg_error < 10 and max_error < 15:
            print("✅ 良好! 標定精度可接受,適合一般抓取任務")
        elif avg_error < 20 and max_error < 30:
            print("⚠️  中等! 標定精度尚可,建議重新標定以提升精度")
        else:
            print("❌ 較差! 建議重新採集標定數據並重新標定")
        
        print("=" * 60)
    
    def save_test_results(self, results, filename='calibration_test_results.json'):
        """儲存測試結果"""
        data = {
            'test_time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'num_tests': len(results),
            'average_error': float(np.mean([r['error'] for r in results])),
            'max_error': float(np.max([r['error'] for r in results])),
            'min_error': float(np.min([r['error'] for r in results])),
            'std_error': float(np.std([r['error'] for r in results])),
            'test_results': results
        }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"\n測試結果已儲存至: {filename}")


def main():
    """主程式"""
    print("=== 相機到機械臂坐標轉換系統 ===")
    
    # 載入標定數據並計算轉換矩陣
    transformer = CameraToRobotTransform(r"C:\Users\user\Downloads\Hand-Eye-Calibration-main\eye_to_hand\aruco_records\complete_record_20260203_112200.json")
    
    # 顯示標定統計
    transformer.print_calibration_stats()
    
    # 視覺化標定結果
    transformer.visualize_calibration()
    
    # 儲存轉換矩陣
    transformer.save_transformation_matrix('camera_to_robot_transform.json')
    
    # 選單
    print("\n" + "=" * 60)
    print("請選擇功能:")
    print("1. 啟動即時轉換系統 (手動測試)")
    print("2. 直接執行自動測試 (推薦)")
    print("3. 退出")
    print("=" * 60)
    
    choice = input("\n請輸入選項 (1/2/3): ").strip()
    
    if choice == '1':
        real_time_system = RealTimeTransform(transformer)
        real_time_system.run()
    elif choice == '2':
        print("\n正在啟動自動測試...")
        real_time_system = RealTimeTransform(transformer)
        real_time_system.auto_test_mode()
        real_time_system.pipeline.stop()
    elif choice == '3':
        print("退出程式")
    else:
        print("無效選項")
    
    print("\n程式結束")


if __name__ == "__main__":
    main()