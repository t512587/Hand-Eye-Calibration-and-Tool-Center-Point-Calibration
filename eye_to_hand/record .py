#!/usr/bin/env python3
"""
Eye-to-Hand 資料記錄程式
相機固定在外部，觀察機械臂末端的標記
收集 15 筆資料後自動結束
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import datetime
from pymycobot.elephantrobot import ElephantRobot
import json
import os
import math


# ========================
# 採樣質量評估類
# ========================
class CalibrationChecker:
    def __init__(self):
        self.samples = []
        self.param_ranges = {
            'rot_x': 60.0, 'rot_y': 60.0, 'rot_z': 60.0,
            'trans_x': 150.0, 'trans_y': 150.0, 'trans_z': 100.0,
            'marker_x': 0.6, 'marker_y': 0.6,
        }
        self._param_names = ['Rot_X', 'Rot_Y', 'Rot_Z', 'Tr_X', 'Tr_Y', 'Tr_Z', 'Mk_X', 'Mk_Y']
        
    def get_parameters(self, robot_coords, corners, image_size):
        """計算當前樣本的參數"""
        rx, ry, rz = robot_coords[3:6] if len(robot_coords) >= 6 else (0, 0, 0)
        px, py, pz = robot_coords[0:3] if len(robot_coords) >= 3 else (0, 0, 0)
        
        corner_points = corners[0] if len(corners.shape) == 3 else corners
        center_x = np.mean(corner_points[:, 0]) / image_size[0]
        center_y = np.mean(corner_points[:, 1]) / image_size[1]
        
        return [rx, ry, rz, px, py, pz, center_x, center_y]
    
    def is_good_sample(self, params):
        """判斷新樣本是否與已有樣本足夠不同（強制要求角度變化）"""
        if len(self.samples) == 0:
            return True
        
        for old_params, _, _ in self.samples:
            rot_diff = math.sqrt(sum((params[i] - old_params[i])**2 for i in range(3)))
            trans_diff = math.sqrt(sum((params[i] - old_params[i])**2 for i in range(3, 6)))
            marker_diff = math.sqrt(sum((params[i] - old_params[i])**2 for i in range(6, 8)))
            
            # 強制要求：角度必須有變化（至少 8 度）
            if rot_diff < 8.0:
                return False  # 角度變化不足，直接拒絕，不管位移多大
            
            # 如果角度夠了，還要確保位移也有一定變化
            if trans_diff < 20.0 and marker_diff < 0.1:
                return False
        
        return True
    
    def add_sample(self, robot_coords, marker_data, corners, image_size):
        """添加新樣本"""
        params = self.get_parameters(robot_coords, corners, image_size)
        if self.is_good_sample(params):
            self.samples.append((params, robot_coords, marker_data))
            return True
        return False
    
    def compute_progress(self):
        """計算當前採樣覆蓋率"""
        if len(self.samples) == 0:
            return None
        
        all_params = [sample[0] for sample in self.samples]
        min_params = [min(p[i] for p in all_params) for i in range(8)]
        max_params = [max(p[i] for p in all_params) for i in range(8)]
        
        ranges = [
            self.param_ranges['rot_x'], self.param_ranges['rot_y'], self.param_ranges['rot_z'],
            self.param_ranges['trans_x'], self.param_ranges['trans_y'], self.param_ranges['trans_z'],
            self.param_ranges['marker_x'], self.param_ranges['marker_y']
        ]
        
        progress = [min(1.0, (max_params[i] - min_params[i]) / ranges[i]) for i in range(8)]
        return list(zip(self._param_names, progress))
    
    def get_suggestion(self):
        """獲取採樣建議"""
        if len(self.samples) == 0:
            return "Start collecting first sample"
        
        progress_data = self.compute_progress()
        if not progress_data:
            return "Start collecting samples"
        
        suggestions = {
            'Rot_X': "Adjust Roll angle",
            'Rot_Y': "Adjust Pitch angle",
            'Rot_Z': "Adjust Yaw angle",
            'Tr_X': "Move robot left/right",
            'Tr_Y': "Move robot forward/backward",
            'Tr_Z': "Move robot up/down",
            'Mk_X': "Move marker to left/right edges",
            'Mk_Y': "Move marker to top/bottom edges"
        }
        
        for name, prog in progress_data:
            if prog < 0.8:
                return f"Suggest: {suggestions.get(name, name)}"
        
        return "Good coverage!"


# ========================
# 繪製進度視窗
# ========================
def draw_progress_window(checker, current_count, target_points):
    """繪製進度視窗"""
    progress_img = np.zeros((650, 500, 3), dtype=np.uint8)
    progress_img[:] = (40, 40, 40)
    
    # 標題
    cv2.putText(progress_img, "Eye-to-Hand Recording", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
    
    # 樣本計數
    count_color = (0, 255, 0) if current_count >= target_points else (255, 255, 255)
    cv2.putText(progress_img, f"Samples: {current_count}/{target_points}", (20, 85), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, count_color, 2)
    
    # 進度條
    progress_data = checker.compute_progress()
    if progress_data:
        bar_width, bar_height, spacing = 350, 30, 50
        start_x, start_y = 20, 130
        
        for i, (name, progress) in enumerate(progress_data):
            y = start_y + i * spacing
            bar_x = start_x + 90
            
            cv2.putText(progress_img, f"{name}:", (start_x, y + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.rectangle(progress_img, (bar_x, y), (bar_x + bar_width, y + bar_height), 
                          (60, 60, 60), -1)
            
            fill_width = int(bar_width * progress)
            color = (0, 255, 0) if progress >= 0.8 else (0, 255, 255) if progress >= 0.5 else (0, 0, 255)
            cv2.rectangle(progress_img, (bar_x, y), (bar_x + fill_width, y + bar_height), 
                          color, -1)
            
            cv2.rectangle(progress_img, (bar_x, y), (bar_x + bar_width, y + bar_height), 
                          (200, 200, 200), 2)
            
            cv2.putText(progress_img, f"{int(progress * 100)}%", (bar_x + bar_width + 12, y + 22), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    else:
        cv2.putText(progress_img, "No samples yet", (20, 250), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)
    
    # 分隔線
    cv2.line(progress_img, (20, 565), (480, 565), (100, 100, 100), 2)
    
    # 建議
    if current_count >= target_points:
        cv2.putText(progress_img, "RECORDING COMPLETE!", (20, 595), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        suggestion = checker.get_suggestion()
        cv2.putText(progress_img, suggestion, (20, 595), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
        cv2.putText(progress_img, "Press 's' to capture", (20, 625), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    
    return progress_img


# ========================
# 主程式
# ========================
def get_camera_intrinsics(pipeline, config):
    """獲取相機內參"""
    profile = pipeline.start(config)
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    intr = color_frame.profile.as_video_stream_profile().get_intrinsics()
    pipeline.stop()
    
    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ])
    return camera_matrix, np.zeros((5, 1))


def main():
    # 連接機械臂
    robot = ElephantRobot("192.168.50.123", 5001)
    robot.start_client()
    print(f"機械臂連接成功，當前坐標: {robot.get_coords()}")

    # 設置相機
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    camera_matrix, dist_coeffs = get_camera_intrinsics(pipeline, config)
    pipeline.start(config)

    # ArUco 設置
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    marker_length = 0.04

    # 初始化
    checker = CalibrationChecker()
    recorded_points = []
    target_points = 20
    current_count = 0
    image_size = (640, 480)

    # 創建存儲文件夾
    os.makedirs('aruco_records', exist_ok=True)

    # 創建視窗
    cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Recording Progress', cv2.WINDOW_NORMAL)
    cv2.moveWindow('Camera View', 50, 50)
    cv2.moveWindow('Recording Progress', 750, 50)

    print("\n" + "="*50)
    print("  Eye-to-Hand 資料記錄系統")
    print("="*50)
    print("需要採集 15 個樣本點")
    print("按 's' 記錄樣本 | 按 'r' 重置 | 按 'q' 退出")
    print("="*50 + "\n")

    try:
        while current_count < target_points:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            display = color_image.copy()

            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, marker_length, camera_matrix, dist_coeffs)

                for i in range(len(ids)):
                    cv2.aruco.drawDetectedMarkers(display, corners)
                    cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, 
                                      rvecs[i], tvecs[i], 0.03)

                # 判斷樣本質量
                robot_coords = robot.get_coords()
                is_good = checker.is_good_sample(
                    checker.get_parameters(robot_coords, corners[0], image_size))
                
                # 顯示質量指示
                x, y = display.shape[1] - 170, 30
                if is_good:
                    cv2.putText(display, "GOOD SAMPLE", (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.circle(display, (x - 15, y - 5), 8, (0, 255, 0), -1)
                else:
                    cv2.putText(display, "TOO SIMILAR", (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.circle(display, (x - 15, y - 5), 8, (0, 0, 255), -1)

                # 顯示標記資訊
                for i in range(len(ids)):
                    corner = corners[i][0]
                    dist = np.linalg.norm(tvecs[i][0]) * 1000
                    cv2.putText(display, f"ID:{ids[i][0]} D:{dist:.0f}mm", 
                               (int(corner[0][0]), int(corner[0][1])-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 顯示檢測狀態
            status = f"Detected: {len(ids)} markers" if ids is not None else "No ArUco detected"
            color = (0, 255, 0) if ids is not None else (0, 0, 255)
            cv2.putText(display, status, (display.shape[1] - 220, display.shape[0] - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 顯示視窗
            cv2.imshow('Camera View', display)
            progress_window = draw_progress_window(checker, current_count, target_points)
            cv2.imshow('Recording Progress', progress_window)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            elif key == ord('s') and ids is not None:
                robot_coords = robot.get_coords()
                
                if checker.add_sample(robot_coords, None, corners[0], image_size):
                    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                    
                    # 建立記錄資料（與 record.py 相同格式）
                    record_data = {
                        'record_id': current_count + 1,
                        'timestamp': timestamp,
                        'robot_coords': robot_coords,
                        'aruco_markers': []
                    }

                    for i in range(len(ids)):
                        tvec_mm = tvecs[i][0] * 1000
                        marker_data = {
                            'id': int(ids[i][0]),
                            'translation_mm': tvec_mm.tolist(),
                            'rotation_vector': rvecs[i][0].tolist(),
                            'corners': corners[i].tolist()
                        }
                        record_data['aruco_markers'].append(marker_data)
                    
                    recorded_points.append(record_data)
                    current_count += 1

                    # 儲存影像
                    cv2.imwrite(f"aruco_records/point_{current_count:02d}_{timestamp}.png", color_image)

                    print(f"\n✓ 記錄點位 {current_count}/{target_points}")
                    print(f"  機械臂: {robot_coords}")
                    print(f"  檢測到 {len(ids)} 個標記")
                else:
                    print("\n⚠ 樣本太相似，請移動機械臂")

            elif key == ord('r'):
                recorded_points = []
                current_count = 0
                checker.samples = []
                print("\n=== 記錄已重置 ===")

        # 完成記錄，儲存 JSON
        if current_count >= target_points:
            print(f"\n🎉 已完成所有 {target_points} 個點位的記錄！")
            json_filename = f"aruco_records/complete_record_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(json_filename, 'w', encoding='utf-8') as f:
                json.dump(recorded_points, f, ensure_ascii=False, indent=2)
            
            print(f"完整記錄已儲存至: {json_filename}")
            
            total_markers = sum(len(point['aruco_markers']) for point in recorded_points)
            print(f"\n總記錄點數: {len(recorded_points)}")
            print(f"總標記數: {total_markers}")
            print(f"平均每點: {total_markers/len(recorded_points):.1f} 個標記")
            
            # 顯示完成畫面 3 秒
            for _ in range(30):
                progress_window = draw_progress_window(checker, current_count, target_points)
                cv2.imshow('Recording Progress', progress_window)
                cv2.waitKey(100)

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\n程式結束")


if __name__ == "__main__":
    main()