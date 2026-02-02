#!/usr/bin/env python3
"""
Eye-in-Hand 資料記錄程式（增強版）
相機固定在機械臂末端，觀察外部固定的棋盤格
包含採樣質量評估、進度視覺化、智能建議等功能
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
# 棋盤格設定
# ========================
CHESSBOARD_SIZE = (9, 6)  # 內角點數量 (列, 行)
SQUARE_SIZE = 0.025  # 每個方格的實際大小，單位: 公尺 (25mm)


# ========================
# 採樣質量評估類
# ========================
class CalibrationChecker:
    def __init__(self):
        self.samples = []
        self.param_ranges = {
            'rot_x': 60.0, 'rot_y': 60.0, 'rot_z': 60.0,
            'trans_x': 150.0, 'trans_y': 150.0, 'trans_z': 100.0,
            'board_x': 0.6, 'board_y': 0.6,
        }
        self._param_names = ['Rot_X', 'Rot_Y', 'Rot_Z', 'Tr_X', 'Tr_Y', 'Tr_Z', 'Bd_X', 'Bd_Y']
        
    def get_parameters(self, robot_coords, corners, image_size):
        """計算當前樣本的參數"""
        rx, ry, rz = robot_coords[3:6] if len(robot_coords) >= 6 else (0, 0, 0)
        px, py, pz = robot_coords[0:3] if len(robot_coords) >= 3 else (0, 0, 0)
        
        # 計算棋盤格在影像中的中心位置
        corner_points = corners.reshape(-1, 2)
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
            board_diff = math.sqrt(sum((params[i] - old_params[i])**2 for i in range(6, 8)))
            
            # 強制要求：角度必須有變化（至少 8 度）
            if rot_diff < 8.0:
                return False  # 角度變化不足，直接拒絕
            
            # 如果角度夠了，還要確保位移也有一定變化
            if trans_diff < 20.0 and board_diff < 0.1:
                return False
        
        return True
    
    def add_sample(self, robot_coords, chessboard_data, corners, image_size):
        """添加新樣本"""
        params = self.get_parameters(robot_coords, corners, image_size)
        if self.is_good_sample(params):
            self.samples.append((params, robot_coords, chessboard_data))
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
            self.param_ranges['board_x'], self.param_ranges['board_y']
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
            'Rot_X': "Adjust Roll angle (rotate around X)",
            'Rot_Y': "Adjust Pitch angle (rotate around Y)",
            'Rot_Z': "Adjust Yaw angle (rotate around Z)",
            'Tr_X': "Move robot left/right",
            'Tr_Y': "Move robot forward/backward",
            'Tr_Z': "Move robot up/down",
            'Bd_X': "View board from left/right sides",
            'Bd_Y': "View board from top/bottom angles"
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
    cv2.putText(progress_img, "Eye-in-Hand Recording", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 165, 0), 2)
    
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
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 1)
        cv2.putText(progress_img, "Press 's' to capture", (20, 625), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
    
    return progress_img


# ========================
# 生成棋盤格 3D 物件點
# ========================
def create_chessboard_points():
    """生成棋盤格的 3D 世界座標點"""
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE
    return objp


# ========================
# 獲取相機內參
# ========================
def get_camera_intrinsics(pipeline, config):
    """獲取相機內參"""
    profile = pipeline.start(config)
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    intr = color_frame.profile.as_video_stream_profile().get_intrinsics()
    
    camera_matrix = np.array([
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ])
    dist_coeffs = np.array(intr.coeffs[:5]).reshape(5, 1) if len(intr.coeffs) >= 5 else np.zeros((5, 1))
    
    return camera_matrix, dist_coeffs


# ========================
# 主程式
# ========================
def main():
    # 連接機械臂
    robot = ElephantRobot("192.168.50.123", 5001)
    robot.start_client()
    print(f"機械臂連接成功，當前坐標: {robot.get_coords()}")

    # 設置相機
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    camera_matrix, dist_coeffs = get_camera_intrinsics(pipeline, config)
    
    print("\n=== 相機內參 ===")
    print(f"相機矩陣:\n{camera_matrix}")
    print(f"畸變係數: {dist_coeffs.flatten()}")

    # 棋盤格設置
    objp = create_chessboard_points()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 初始化
    checker = CalibrationChecker()
    recorded_points = []
    target_points = 40
    current_count = 0
    image_size = (640, 480)

    # 創建存儲文件夾
    save_dir = 'handeye_records'
    os.makedirs(save_dir, exist_ok=True)

    # 創建視窗
    cv2.namedWindow('Camera View', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Recording Progress', cv2.WINDOW_NORMAL)
    cv2.moveWindow('Camera View', 50, 50)
    cv2.moveWindow('Recording Progress', 750, 50)

    print("\n" + "="*60)
    print("  Eye-in-Hand 資料記錄系統（增強版）")
    print("="*60)
    print(f"棋盤格設定: {CHESSBOARD_SIZE[0]}x{CHESSBOARD_SIZE[1]}")
    print(f"方格大小: {SQUARE_SIZE * 1000:.1f} mm")
    print(f"需要採集 {target_points} 個樣本點")
    print("按 's' 記錄樣本 | 按 'r' 重置 | 按 'q' 退出")
    print("="*60 + "\n")

    try:
        while current_count < target_points:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            display = color_image.copy()

            # 檢測棋盤格角點
            ret, corners = cv2.findChessboardCorners(
                gray, CHESSBOARD_SIZE,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
            )

            if ret:
                # 優化角點位置
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # 繪製棋盤格角點
                cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners_refined, ret)

                # 使用 solvePnP 計算姿態
                success, rvec, tvec = cv2.solvePnP(
                    objp, corners_refined, camera_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if success:
                    # 繪製座標軸
                    cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, SQUARE_SIZE * 3)

                    # 顯示位置資訊
                    tvec_mm = tvec.flatten() * 1000
                    cv2.putText(display, f"Board: ({tvec_mm[0]:.0f}, {tvec_mm[1]:.0f}, {tvec_mm[2]:.0f}) mm",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # 判斷樣本質量
                    robot_coords = robot.get_coords()
                    is_good = checker.is_good_sample(
                        checker.get_parameters(robot_coords, corners_refined, image_size))
                    
                    # 顯示質量指示
                    x, y = display.shape[1] - 170, 60
                    if is_good:
                        cv2.putText(display, "GOOD SAMPLE", (x, y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.circle(display, (x - 15, y - 5), 8, (0, 255, 0), -1)
                    else:
                        cv2.putText(display, "TOO SIMILAR", (x, y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        cv2.circle(display, (x - 15, y - 5), 8, (0, 0, 255), -1)

            # 顯示檢測狀態
            status = "Chessboard DETECTED" if ret else "Chessboard NOT found"
            color = (0, 255, 0) if ret else (0, 0, 255)
            cv2.putText(display, status, (10, display.shape[0] - 45), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 顯示樣本計數
            cv2.putText(display, f"Samples: {current_count}/{target_points}", 
                       (10, display.shape[0] - 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # 顯示視窗
            cv2.imshow('Camera View', display)
            progress_window = draw_progress_window(checker, current_count, target_points)
            cv2.imshow('Recording Progress', progress_window)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            elif key == ord('s') and ret:
                robot_coords = robot.get_coords()
                
                # 重新計算姿態確保數據正確
                success, rvec, tvec = cv2.solvePnP(
                    objp, corners_refined, camera_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                
                if success and checker.add_sample(robot_coords, None, corners_refined, image_size):
                    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                    
                    # 建立記錄資料
                    record_data = {
                        'record_id': current_count + 1,
                        'timestamp': timestamp,
                        'robot_coords': robot_coords,
                        'type': 'chessboard',
                        'chessboard_size': list(CHESSBOARD_SIZE),
                        'square_size_mm': SQUARE_SIZE * 1000,
                        'chessboard_data': {
                            'translation_mm': (tvec.flatten() * 1000).tolist(),
                            'rotation_vector': rvec.flatten().tolist(),
                            'corners': corners_refined.tolist()
                        }
                    }
                    
                    recorded_points.append(record_data)
                    current_count += 1

                    # 儲存影像
                    cv2.imwrite(f"{save_dir}/point_{current_count:02d}_{timestamp}.png", color_image)

                    print(f"\n✓ 記錄點位 {current_count}/{target_points}")
                    print(f"  機械臂: {robot_coords}")
                    print(f"  棋盤格位置: {tvec_mm.tolist()}")
                else:
                    print("\n⚠ 樣本太相似，請移動機械臂到不同姿態")

            elif key == ord('r'):
                recorded_points = []
                current_count = 0
                checker.samples = []
                print("\n=== 記錄已重置 ===")

        # 完成記錄，儲存 JSON
        if current_count >= target_points:
            print(f"\n🎉 已完成所有 {target_points} 個點位的記錄！")
            json_filename = f"{save_dir}/handeye_chessboard_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(json_filename, 'w', encoding='utf-8') as f:
                json.dump(recorded_points, f, ensure_ascii=False, indent=2)
            
            print(f"完整記錄已儲存至: {json_filename}")
            
            print(f"\n總記錄點數: {len(recorded_points)}")
            print(f"棋盤格設定: {CHESSBOARD_SIZE[0]}x{CHESSBOARD_SIZE[1]}")
            print(f"方格大小: {SQUARE_SIZE * 1000:.1f} mm")
            
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