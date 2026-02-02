#!/usr/bin/env python3
"""
YOLO 物件檢測 + 手眼標定 + 機械臂抓取系統 (6D 姿勢版本)
- 讀取手眼標定 JSON
- 使用 YOLO 模型進行物件檢測
- RealSense 深度相機獲取 3D 座標
- 轉換相機座標到機械臂座標
- 控制機械臂移動到目標物件（包含姿勢控制）
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import json
from pymycobot.elephantrobot import ElephantRobot,JogMode
from ultralytics import YOLO
import time


class CameraToRobotTransformer:
    """相機座標到機械臂座標轉換器"""
    
    def __init__(self, json_file):
        """載入手眼標定結果
        
        Args:
            json_file: 標定 JSON 檔案路徑
        """
        with open(json_file, 'r') as f:
            calib_data = json.load(f)
        
        self.T_cam2robot = np.array(calib_data['transformation_matrix'])
        self.camera_matrix = np.array(calib_data['camera_matrix'])
        print("✓ 已載入手眼標定資料")
        print(f"  RMSE 誤差: {calib_data.get('rmse_error', 'N/A')} mm")
    
    def transform_point(self, camera_point):
        """將相機座標轉換為機械臂座標
        
        Args:
            camera_point: 相機座標系中的點 [x, y, z] (單位: mm)
        
        Returns:
            機械臂座標系中的點 [x, y, z] (單位: mm)
        """
        # 相機座標點 (齊次座標)
        point_cam = np.array([
            camera_point[0],
            camera_point[1],
            camera_point[2],
            1.0
        ])
        
        # 轉換到機械臂座標系
        point_robot = self.T_cam2robot @ point_cam
        
        return point_robot[:3]


class YOLODetector:
    """YOLO 物件檢測器"""
    
    def __init__(self, model_path):
        """初始化 YOLO 模型
        
        Args:
            model_path: YOLO 模型權重檔案路徑
        """
        self.model = YOLO(model_path)
        print(f"✓ 已載入 YOLO 模型: {model_path}")
    
    def detect(self, image, conf_threshold=0.5):
        """執行物件檢測
        
        Args:
            image: 輸入影像 (BGR)
            conf_threshold: 信心度閾值
        
        Returns:
            檢測結果列表，每個結果包含位置、尺寸、類別等資訊
        """
        results = self.model(image, conf=conf_threshold, verbose=False)
        
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = result.names[cls_id]
                
                # 計算中心點
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)
                width = int(x2 - x1)
                height = int(y2 - y1)
                
                detections.append({
                    'center': (x_center, y_center),
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'width': width,
                    'height': height,
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': cls_name
                })
        
        return detections
    
    def draw_detections(self, image, detections):
        """在影像上繪製檢測結果
        
        Args:
            image: 輸入影像
            detections: 檢測結果列表
        
        Returns:
            繪製後的影像
        """
        output = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            x_center, y_center = det['center']
            conf = det['confidence']
            cls_name = det['class_name']
            
            # 繪製邊界框
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 繪製中心點
            cv2.circle(output, (x_center, y_center), 5, (0, 0, 255), -1)
            
            # 繪製標籤
            label = f"{cls_name} {conf:.2f}"
            cv2.putText(output, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return output


class RealSenseCamera:
    """RealSense 深度相機管理器"""
    
    def __init__(self, width=1280, height=720, fps=30):
        """初始化 RealSense 相機
        
        Args:
            width: 影像寬度
            height: 影像高度
            fps: 幀率
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 啟用深度和彩色串流
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # 啟動相機
        self.profile = self.pipeline.start(self.config)
        
        # 獲取深度比例尺
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # 對齊深度到彩色影像
        self.align = rs.align(rs.stream.color)
        
        print(f"✓ RealSense 相機已啟動 ({width}x{height} @ {fps}fps)")
        print(f"  深度比例尺: {self.depth_scale}")
    
    def get_frames(self):
        """獲取對齊後的彩色和深度影像
        
        Returns:
            color_image: BGR 彩色影像
            depth_image: 深度影像
            depth_frame: 深度幀 (用於獲取 3D 座標)
        """
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return None, None, None
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_image, depth_image, depth_frame
    
    def get_3d_point(self, depth_frame, pixel_x, pixel_y):
        """從像素座標獲取 3D 座標
        
        Args:
            depth_frame: 深度幀
            pixel_x: 像素 x 座標
            pixel_y: 像素 y 座標
        
        Returns:
            3D 座標 [x, y, z] (單位: mm)，如果深度無效則返回 None
        """
        depth = depth_frame.get_distance(pixel_x, pixel_y)
        
        if depth == 0:
            return None
        
        # 獲取深度內參
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        
        # 反投影到 3D 空間
        point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [pixel_x, pixel_y], depth)
        
        # 轉換為 mm
        point_3d_mm = [coord * 1000 for coord in point_3d]
        
        return point_3d_mm
    
    def stop(self):
        """停止相機"""
        self.pipeline.stop()


class RobotPickSystem:
    """機械臂抓取系統（6D 姿勢版本）"""
    
    def __init__(self, robot_ip, robot_port, calib_json, yolo_model):
        """初始化系統
        
        Args:
            robot_ip: 機械臂 IP
            robot_port: 機械臂埠口
            calib_json: 手眼標定 JSON 檔案
            yolo_model: YOLO 模型路徑
        """
        # 初始化機械臂
        self.robot = ElephantRobot(robot_ip, robot_port)
        self.robot.start_client()
        print(f"✓ 機械臂已連接: {robot_ip}:{robot_port}")
        print(f"  當前座標: {self.robot.get_coords()}")
        
        # 初始化座標轉換器
        self.transformer = CameraToRobotTransformer(calib_json)
        
        # 初始化 YOLO 檢測器
        self.detector = YOLODetector(yolo_model)
        
        # 初始化相機
        self.camera = RealSenseCamera()
        
        # 機械臂移動速度
        self.coords_speed = 2000
        
        # 預設末端執行器姿勢（6D 姿勢）
        self.default_pose = {
            'rx': 90.0,   # Roll (繞 X 軸旋轉)
            'ry': 20.0,    # Pitch (繞 Y 軸旋轉)
            'rz': 90.0    # Yaw (繞 Z 軸旋轉)
        }
        
        print(f"  預設姿勢: RX={self.default_pose['rx']}°, RY={self.default_pose['ry']}°, RZ={self.default_pose['rz']}°")
    
    def set_default_pose(self, rx=None, ry=None, rz=None):
        """設定預設末端執行器姿勢
        
        Args:
            rx: Roll 角度（繞 X 軸）
            ry: Pitch 角度（繞 Y 軸）
            rz: Yaw 角度（繞 Z 軸）
        """
        if rx is not None:
            self.default_pose['rx'] = rx
        if ry is not None:
            self.default_pose['ry'] = ry
        if rz is not None:
            self.default_pose['rz'] = rz
        
        print(f"  姿勢已更新: RX={self.default_pose['rx']}°, RY={self.default_pose['ry']}°, RZ={self.default_pose['rz']}°")
    
    def select_target(self, detections):
        """選擇目標物件（可自定義選擇邏輯）
        
        Args:
            detections: 檢測結果列表
        
        Returns:
            選中的目標，如果沒有檢測到則返回 None
        """
        if len(detections) == 0:
            return None
        
        # 預設選擇信心度最高的物件
        target = max(detections, key=lambda x: x['confidence'])
        return target
    
    def move_to_object_6d(self, robot_coords, offset_z=50, rx=None, ry=None, rz=None):
        """移動機械臂到物件位置(6D 姿勢控制)並執行抓取
        
        Args:
            robot_coords: 機械臂座標系中的目標位置 [x, y, z]
            offset_z: Z 軸偏移量 (mm),用於懸停在物件上方
            rx: 末端執行器 Roll 角度(如果為 None 則使用預設值)
            ry: 末端執行器 Pitch 角度(如果為 None 則使用預設值)
            rz: 末端執行器 Yaw 角度(如果為 None 則使用預設值)
        
        Returns:
            是否成功移動
        """
        
        # 使用指定姿勢或預設姿勢
        target_rx = rx if rx is not None else self.default_pose['rx']
        target_ry = ry if ry is not None else self.default_pose['ry']
        target_rz = rz if rz is not None else self.default_pose['rz']
        
        # 階段 1: 移動到 X-170 位置(準備抓取位置)
        prepare_coords = [
            robot_coords[0] - 190,      # X 位置 - 170
            robot_coords[1],            # Y 位置
            robot_coords[2] + offset_z, # Z 位置(加上偏移量懸停)
            target_rx,                  # RX 姿勢
            target_ry,                  # RY 姿勢
            target_rz                   # RZ 姿勢
        ]
        
        print(f"  準備抓取位置 (X-170):")
        print(f"    位置: X={prepare_coords[0]:.1f}, Y={prepare_coords[1]:.1f}, Z={prepare_coords[2]:.1f} mm")
        print(f"    姿勢: RX={prepare_coords[3]:.1f}°, RY={prepare_coords[4]:.1f}°, RZ={prepare_coords[5]:.1f}°")
        
        # 發送移動指令到準備位置
        self.robot.write_coords(prepare_coords, self.coords_speed)
        
        # 等待到達準備位置
        print("  等待到達準備位置...")
        time.sleep(0.5)
        timeout = 10  # 10秒超時
        start_time = time.time()
        
        while not self.robot.is_in_position(prepare_coords, JogMode.JOG_TELEOP):
            if time.time() - start_time > timeout:
                print("  ⚠ 移動超時!")
                return False
            time.sleep(0.1)
        
        print("  ✓ 已到達準備位置")
        
        # 打開夾爪
        print("  打開夾爪...")
        self.robot.set_digital_out(16, 0)  # IO 16 設為 1
        self.robot.set_digital_out(17, 1)  # IO 17 設為 0
        time.sleep(1.5)  # 等待夾爪動作完成
        
        # 階段 2: 移動到最終目標位置
        final_coords = [
            robot_coords[0] - 100,      # X 位置 - 100
            robot_coords[1],            # Y 位置
            robot_coords[2] + offset_z, # Z 位置
            target_rx,                  # RX 姿勢
            target_ry,                  # RY 姿勢
            target_rz                   # RZ 姿勢
        ]
        
        print(f"  繼續移動到最終位置:")
        print(f"    位置: X={final_coords[0]:.1f}, Y={final_coords[1]:.1f}, Z={final_coords[2]:.1f} mm")
        
        # 發送移動指令到最終位置
        self.robot.write_coords(final_coords, self.coords_speed)
        
        # 等待到達最終位置
        print("  等待到達最終位置...")
        time.sleep(0.5)
        start_time = time.time()
        
        while not self.robot.is_in_position(final_coords, JogMode.JOG_TELEOP):
            if time.time() - start_time > timeout:
                print("  ⚠ 移動超時!")
                return False
            time.sleep(0.1)
        
        print("  ✓ 移動完成")
        print("  打開夾爪...")
        self.robot.set_digital_out(16, 1)  # IO 16 設為 1
        self.robot.set_digital_out(17, 0)  # IO 17 設為 0
        time.sleep(1.5)  # 等待夾爪動作完成
        return True
    
    def run(self):
        """執行主迴圈"""
        print("\n" + "="*60)
        print("  YOLO 物件檢測 + 機械臂抓取系統 (6D 姿勢版本)")
        print("="*60)
        print("按 's' 選擇並移動到目標物件")
        print("按 'p' 更改末端執行器姿勢")
        print("按 'q' 退出")
        print("="*60 + "\n")
        
        try:
            while True:
                # 獲取影像
                color_image, depth_image, depth_frame = self.camera.get_frames()
                
                if color_image is None:
                    continue
                
                # 執行物件檢測
                detections = self.detector.detect(color_image)
                
                # 繪製檢測結果
                display_image = self.detector.draw_detections(color_image, detections)
                
                # 顯示檢測數量和當前姿勢
                cv2.putText(display_image, f"Detected: {len(detections)} objects", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.putText(display_image, 
                           f"Pose: RX={self.default_pose['rx']:.0f} RY={self.default_pose['ry']:.0f} RZ={self.default_pose['rz']:.0f}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # 深度影像視覺化
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                
                # 並排顯示
                combined = np.hstack((display_image, depth_colormap))
                cv2.imshow('YOLO Detection + Depth (6D Pose)', combined)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                
                elif key == ord('p'):
                    # 更改姿勢
                    print("\n請輸入新的末端執行器姿勢:")
                    try:
                        rx = float(input(f"  RX (當前: {self.default_pose['rx']}°): ") or self.default_pose['rx'])
                        ry = float(input(f"  RY (當前: {self.default_pose['ry']}°): ") or self.default_pose['ry'])
                        rz = float(input(f"  RZ (當前: {self.default_pose['rz']}°): ") or self.default_pose['rz'])
                        self.set_default_pose(rx, ry, rz)
                    except ValueError:
                        print("⚠ 輸入無效，保持原姿勢")
                
                elif key == ord('s'):
                    # 選擇目標
                    target = self.select_target(detections)
                    
                    if target is None:
                        print("⚠ 未檢測到物件")
                        continue
                    
                    print(f"\n目標物件: {target['class_name']} (信心度: {target['confidence']:.2f})")
                    
                    # 獲取目標中心的 3D 座標
                    x_pixel, y_pixel = target['center']
                    camera_point = self.camera.get_3d_point(depth_frame, x_pixel, y_pixel)
                    
                    if camera_point is None:
                        print("⚠ 無法獲取深度資訊")
                        continue
                    
                    print(f"  相機座標: X={camera_point[0]:.1f}, Y={camera_point[1]:.1f}, Z={camera_point[2]:.1f} mm")
                    
                    # 轉換到機械臂座標
                    robot_coords = self.transformer.transform_point(camera_point)
                    print(f"  機械臂座標: X={robot_coords[0]:.1f}, Y={robot_coords[1]:.1f}, Z={robot_coords[2]:.1f} mm")
                    
                    # 移動機械臂（使用 6D 姿勢）
                    print("  開始移動機械臂...")
                    self.move_to_object_6d(robot_coords, offset_z=0)
                    
                    print("  按 's' 繼續選擇下一個目標\n")
        
        finally:
            self.camera.stop()
            cv2.destroyAllWindows()
            print("\n系統已關閉")


def main():
    """主程式"""
    # 配置參數
    ROBOT_IP = "192.168.50.123"
    ROBOT_PORT = 5001
    CALIB_JSON = "camera_to_robot_transform.json"  # 手眼標定檔案
    YOLO_MODEL = "weights/best.pt"  # YOLO 模型檔案
    
    # 啟動系統
    system = RobotPickSystem(
        robot_ip=ROBOT_IP,
        robot_port=ROBOT_PORT,
        calib_json=CALIB_JSON,
        yolo_model=YOLO_MODEL
    )
    
    # 可以在這裡自訂預設姿勢（選擇性）
    # system.set_default_pose(rx=90, ry=0, rz=90)
    
    system.run()


if __name__ == "__main__":
    main()