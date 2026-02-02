from pymycobot.elephantrobot import ElephantRobot, JogMode
import time
import json
import os
import numpy as np
from datetime import datetime

class TCPAutoCalibration:
    """TCP 自動校正系統 - 使用法蘭座標系，單次圓形掃描"""
    
    def __init__(self, host, port):
        self.robot = ElephantRobot(host, port)
        self.robot.start_client()
        
        # 雷射訊號接腳
        self.LASER_X_PIN, self.LASER_Y_PIN = 0, 1
        
        # 預設法蘭座標系參數（不會被修改）
        self.DEFAULT_TCP_TARGET = {'X': 214.221, 'Y': -60.728, 'Z': 224.247}
        self.FLANGE_POSE = {'RX': 179.0, 'RY': 0.0, 'RZ': 120.0}
        
        # 工具與校正參數
        self.TOOL_DIAMETER = 40.0
        self.TOOL_RADIUS = self.TOOL_DIAMETER / 2.0
        self.TOLERANCE_XY, self.TOLERANCE_Z = 0.3, 0.2
        self.SCAN_RADIUS, self.SCAN_POINTS = 15.0, 72
        self.SPEED_FAST = self.SPEED_SCAN = self.SPEED_FINE = self.SPEED_Z_SEARCH = 500
        self.SAFE_HEIGHT = 20.0
        self.TRIGGER_STABLE_COUNT = 5
        self.TRIGGER_CHECK_INTERVAL = 0.05
        self.POSITION_CHECK_INTERVAL = 0.2
        self.MAX_WAIT_TIME = 30.0
        
        # 偏移量檔案
        self.OFFSET_FILE = 'tcp_offset.json'
        
        # 載入偏移量並計算當前TCP
        self.tcp_offset = self.load_offset()
        self.TCP_TARGET = self.apply_offset()
        
        # 校正結果
        self.current_result = {}
        
        print(f"✓ 當前TCP: X={self.TCP_TARGET['X']:.3f}, Y={self.TCP_TARGET['Y']:.3f}, Z={self.TCP_TARGET['Z']:.3f}")
        if any(abs(v) > 0.001 for v in self.tcp_offset.values()):
            print(f"  偏移量: ΔX={self.tcp_offset['X']:+.3f}, ΔY={self.tcp_offset['Y']:+.3f}, ΔZ={self.tcp_offset['Z']:+.3f}")
    
    # ==================== 偏移量管理 ====================
    
    def load_offset(self):
        """載入TCP偏移量"""
        try:
            if os.path.exists(self.OFFSET_FILE):
                with open(self.OFFSET_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                if 'offset' in data:
                    print(f"\n✓ 載入上次校正 (時間: {data.get('time', '未知')})")
                    return data['offset']
        except Exception as e:
            print(f"\n⚠️ 載入偏移量失敗: {e}")
        return {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
    
    def apply_offset(self):
        """應用偏移量到預設TCP"""
        return {
            'X': self.DEFAULT_TCP_TARGET['X'] + self.tcp_offset['X'],
            'Y': self.DEFAULT_TCP_TARGET['Y'] + self.tcp_offset['Y'],
            'Z': self.DEFAULT_TCP_TARGET['Z'] + self.tcp_offset['Z']
        }
    
    def save_offset(self, delta):
        """保存TCP偏移量"""
        try:
            data = {
                'offset': delta,
                'time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'default_tcp': self.DEFAULT_TCP_TARGET,
                'calibrated_tcp': self.TCP_TARGET
            }
            with open(self.OFFSET_FILE, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            print(f"\n✓ 偏移量已保存到 {self.OFFSET_FILE}")
            return True
        except Exception as e:
            print(f"\n⚠️ 保存偏移量失敗: {e}")
            return False
    
    # ==================== 移動控制 ====================
    
    def wait_for_position(self, target_coords, timeout=30, show_progress=True):
        """等待機器人到達目標位置"""
        if isinstance(target_coords, np.ndarray):
            target_coords = target_coords.tolist()
        
        start_time = last_print_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                if self.robot.is_in_position(target_coords, JogMode.JOG_TELEOP):
                    if show_progress:
                        print(f"    ✓ 已到位 (耗時: {time.time()-start_time:.1f}s)")
                    return True
                
                if show_progress and (time.time() - start_time - last_print_time) >= 1.0:
                    print(f"    等待中... {time.time()-start_time:.1f}s")
                    last_print_time = time.time() - start_time
            except Exception as e:
                print(f"    ⚠️ 檢查位置錯誤: {e}")
            
            time.sleep(self.POSITION_CHECK_INTERVAL)
        
        try:
            current = self.robot.get_coords()
            print(f"    ⚠️ 移動超時\n    目標: {target_coords[:3]}\n    當前: {current[:3]}")
        except:
            print(f"    ⚠️ 移動超時且無法讀取當前位置")
        return False
    
    def move_to_position(self, target_coords, speed, timeout=30, show_progress=True):
        """移動到目標位置並等待到位"""
        if isinstance(target_coords, list):
            target_coords = np.array(target_coords, dtype=float)
        self.robot.write_coords(target_coords, speed)
        time.sleep(0.1)
        return self.wait_for_position(target_coords, timeout, show_progress)
    
    # ==================== 雷射檢測 ====================
    
    def check_laser(self, pin):
        """檢查雷射狀態"""
        try:
            return self.robot.get_digital_in(pin) == 1
        except:
            return False
    
    def check_laser_x(self):
        return self.check_laser(self.LASER_X_PIN)
    
    def check_laser_y(self):
        return self.check_laser(self.LASER_Y_PIN)
    
    def check_both_lasers(self):
        return self.check_laser_x(), self.check_laser_y()
    
    def wait_for_stable_trigger(self, axis='both', timeout=30):
        """等待穩定的雷射觸發"""
        start_time = time.time()
        trigger_count = 0
        
        while (time.time() - start_time) < timeout:
            if axis == 'x':
                triggered = self.check_laser_x()
            elif axis == 'y':
                triggered = self.check_laser_y()
            else:  # both
                x_blocked, y_blocked = self.check_both_lasers()
                triggered = x_blocked and y_blocked
            
            trigger_count = trigger_count + 1 if triggered else 0
            if trigger_count >= self.TRIGGER_STABLE_COUNT:
                return True
            
            time.sleep(self.TRIGGER_CHECK_INTERVAL)
        return False
    
    # ==================== 快速檢查 ====================
    
    def quick_check(self):
        """快速檢查TCP是否偏移"""
        print("\n" + "="*70)
        print("  快速檢查")
        print("="*70)
        
        result = {'passed': False, 'x_triggered': False, 'y_triggered': False, 
                  'z_error': None, 'need_recalibration': False}
        
        # 移動到理論中心上方
        print(f"\n→ 移動到檢查點上方...")
        check_point = np.array([
            self.TCP_TARGET['X'], self.TCP_TARGET['Y'], 
            self.TCP_TARGET['Z'] + self.SAFE_HEIGHT,
            self.FLANGE_POSE['RX'], self.FLANGE_POSE['RY'], self.FLANGE_POSE['RZ']
        ], dtype=float)
        
        if not self.move_to_position(check_point, self.SPEED_FAST, timeout=30):
            print(f"  ✗ 移動失敗")
            result['need_recalibration'] = True
            return result
        
        time.sleep(0.5)
        
        # 下降到理論Z平面
        print(f"→ 下降到檢查點...")
        target_point = check_point.copy()
        target_point[2] = self.TCP_TARGET['Z']
        
        if not self.move_to_position(target_point, self.SPEED_Z_SEARCH, timeout=30):
            print(f"  ✗ 下降失敗")
            result['need_recalibration'] = True
            return result
        
        time.sleep(0.3)
        
        # 檢查雷射狀態
        x_blocked, y_blocked = self.check_both_lasers()
        result['x_triggered'] = x_blocked
        result['y_triggered'] = y_blocked
        
        print(f"\n雷射狀態:")
        print(f"  X軸: {'✓ 觸發' if x_blocked else '✗ 未觸發'}")
        print(f"  Y軸: {'✓ 觸發' if y_blocked else '✗ 未觸發'}")
        
        if x_blocked and y_blocked:
            print(f"\n✓ 快速檢查通過! TCP位置正確")
            result['passed'] = True
            result['z_error'] = 0.0
        else:
            print(f"\n✗ 快速檢查未通過，需要完整校正")
            result['need_recalibration'] = True
        
        return result
    
    # ==================== 完整校正 ====================
    
    def full_calibration(self):
        """執行完整校正流程"""
        print("\n" + "="*70)
        print("  完整校正 (XY圓形掃描)")
        print("="*70)
        
        result = {'success': False, 'x_center': None, 'y_center': None, 'z_height': None,
                  'delta': {}}
        
        # Z軸校正
        z_result = self._calibrate_z()
        if not z_result['success']:
            return result
        
        result['z_height'] = z_result['z_height']
        
        # XY圓形掃描
        xy_result = self._circular_scan_xy(result['z_height'])
        if not xy_result['success']:
            return result
        
        result['x_center'] = xy_result['x_center']
        result['y_center'] = xy_result['y_center']
        
        # 計算偏差
        result['delta'] = {
            'X': result['x_center'] - self.TCP_TARGET['X'],
            'Y': result['y_center'] - self.TCP_TARGET['Y'],
            'Z': result['z_height'] - self.TCP_TARGET['Z']
        }
        
        result['success'] = True
        
        # 更新偏移量和TCP
        old_offset = self.tcp_offset.copy()
        self.tcp_offset = {
            'X': self.tcp_offset['X'] + result['delta']['X'],
            'Y': self.tcp_offset['Y'] + result['delta']['Y'],
            'Z': self.tcp_offset['Z'] + result['delta']['Z']
        }
        self.TCP_TARGET = self.apply_offset()
        
        # 保存新偏移量
        self.save_offset(self.tcp_offset)
        
        # 顯示結果
        print("\n" + "="*70)
        print("  校正結果")
        print("="*70)
        print(f"\n測得TCP: X={result['x_center']:.3f}, Y={result['y_center']:.3f}, Z={result['z_height']:.3f}")
        print(f"\n本次偏差:")
        print(f"  ΔX = {result['delta']['X']:>+8.3f} mm")
        print(f"  ΔY = {result['delta']['Y']:>+8.3f} mm")
        print(f"  ΔZ = {result['delta']['Z']:>+8.3f} mm")
        print(f"\n累計偏移量:")
        print(f"  ΔX = {self.tcp_offset['X']:>+8.3f} mm (原 {old_offset['X']:+.3f})")
        print(f"  ΔY = {self.tcp_offset['Y']:>+8.3f} mm (原 {old_offset['Y']:+.3f})")
        print(f"  ΔZ = {self.tcp_offset['Z']:>+8.3f} mm (原 {old_offset['Z']:+.3f})")
        print(f"\n新TCP目標:")
        print(f"  X = {self.TCP_TARGET['X']:>8.3f} mm")
        print(f"  Y = {self.TCP_TARGET['Y']:>8.3f} mm")
        print(f"  Z = {self.TCP_TARGET['Z']:>8.3f} mm")
        print("="*70)
        
        return result
    
    def _calibrate_z(self):
        """Z軸校正"""
        print(f"\n→ Step 1: Z軸高度校正")
        result = {'success': False, 'z_height': None}
        
        # 從安全高度開始
        search_point = np.array([
            self.TCP_TARGET['X'], self.TCP_TARGET['Y'], 
            self.TCP_TARGET['Z'] + self.SAFE_HEIGHT,
            self.FLANGE_POSE['RX'], self.FLANGE_POSE['RY'], self.FLANGE_POSE['RZ']
        ], dtype=float)
        
        if not self.move_to_position(search_point, self.SPEED_FAST, timeout=20):
            print(f"  ✗ 移動失敗")
            return result
        
        time.sleep(0.3)
        
        # 下降尋找Z平面
        print(f"  下降搜尋雷射交點...")
        target_point = search_point.copy()
        target_point[2] = self.TCP_TARGET['Z']
        
        self.robot.write_coords(target_point, self.SPEED_Z_SEARCH)
        time.sleep(0.1)
        
        if self.wait_for_stable_trigger('both', timeout=20):
            current_coords = self.robot.get_coords()
            # 停止機器人
            self.robot.write_coords(current_coords, self.SPEED_FAST)
            time.sleep(0.2)
            
            result['z_height'] = current_coords[2]
            result['success'] = True
            print(f"  ✓ 找到Z平面: Z = {result['z_height']:.3f} mm")
        else:
            print(f"  ✗ Z軸搜尋超時")
        
        return result
    
    def _circular_scan_xy(self, current_z):
        """單次圓形掃描同時檢測X和Y軸"""
        print(f"\n→ Step 2: XY平面圓形掃描")
        result = {'success': False, 'x_center': None, 'y_center': None}
        
        center_x = self.TCP_TARGET['X']
        center_y = self.TCP_TARGET['Y']
        
        # 移動到起始點
        start_point = np.array([
            center_x + self.SCAN_RADIUS,
            center_y,
            current_z,
            self.FLANGE_POSE['RX'], self.FLANGE_POSE['RY'], self.FLANGE_POSE['RZ']
        ], dtype=float)
        
        if not self.move_to_position(start_point, self.SPEED_FAST, timeout=20):
            print(f"  ✗ 移動失敗")
            return result
        
        time.sleep(0.3)
        
        # 圓形掃描
        print(f"  開始圓形掃描 (半徑={self.SCAN_RADIUS:.1f}mm, {self.SCAN_POINTS}點)...")
        
        x_line_points = []  # X軸雷射線上的點
        y_line_points = []  # Y軸雷射線上的點
        
        for i in range(self.SCAN_POINTS + 1):
            angle = (360.0 / self.SCAN_POINTS) * i
            
            scan_x = center_x + self.SCAN_RADIUS * np.cos(np.radians(angle))
            scan_y = center_y + self.SCAN_RADIUS * np.sin(np.radians(angle))
            
            scan_point = np.array([scan_x, scan_y, current_z,
                                   self.FLANGE_POSE['RX'], self.FLANGE_POSE['RY'], 
                                   self.FLANGE_POSE['RZ']], dtype=float)
            
            self.robot.write_coords(scan_point, self.SPEED_SCAN)
            
            if not self.wait_for_position(scan_point, timeout=15, show_progress=False):
                continue
            
            x_blocked, y_blocked = self.check_both_lasers()
            
            # 記錄觸發雷射的位置
            if x_blocked:
                x_line_points.append({'x': scan_x, 'y': scan_y})
            if y_blocked:
                y_line_points.append({'x': scan_x, 'y': scan_y})
            
            if (i + 1) % 12 == 0:
                print(f"    進度: {i+1}/{self.SCAN_POINTS+1} "
                      f"(X觸發:{len(x_line_points)}, Y觸發:{len(y_line_points)})")
        
        print(f"\n  掃描完成! X軸:{len(x_line_points)}點, Y軸:{len(y_line_points)}點")
        
        # 計算中心點：X軸雷射線定義Y座標，Y軸雷射線定義X座標
        if len(x_line_points) >= 1 and len(y_line_points) >= 1:
            # X軸雷射是垂直線，觸發時定義Y座標
            y_coords_from_x_laser = [p['y'] for p in x_line_points]
            result['y_center'] = sum(y_coords_from_x_laser) / len(y_coords_from_x_laser)
            
            # Y軸雷射是水平線，觸發時定義X座標
            x_coords_from_y_laser = [p['x'] for p in y_line_points]
            result['x_center'] = sum(x_coords_from_y_laser) / len(x_coords_from_y_laser)
            
            result['success'] = True
            print(f"  ✓ 中心點: X={result['x_center']:.3f}, Y={result['y_center']:.3f}")
        else:
            print(f"  ✗ 觸發點不足")
        
        return result
    
    # ==================== 主流程 ====================
    
    def run_calibration(self, force_full=False):
        """執行校正流程"""
        print("\n" + "="*70)
        print("  TCP 自動校正系統")
        print("="*70)
        
        start_time = time.time()
        
        # 快速檢查
        if not force_full:
            quick_result = self.quick_check()
            
            if quick_result['passed']:
                elapsed = time.time() - start_time
                print(f"\n✓ 檢查通過 (耗時 {elapsed:.1f}s)")
                return {'mode': 'quick_check', 'result': quick_result}
            
            elif not quick_result['need_recalibration']:
                return {'mode': 'quick_check', 'result': quick_result}
        
        # 完整校正
        full_result = self.full_calibration()
        
        if full_result['success']:
            print(f"\n→ 驗證校正結果...")
            verify_result = self.quick_check()
            elapsed = time.time() - start_time
            
            if verify_result['passed']:
                print(f"\n✓ 校正成功! (耗時 {elapsed:.1f}s)")
            else:
                print(f"\n⚠️ 校正完成但驗證未通過 (耗時 {elapsed:.1f}s)")
            
            return {'mode': 'full', 'result': full_result, 'verify': verify_result}
        else:
            print(f"\n✗ 校正失敗")
            return {'mode': 'full', 'result': full_result}
    
    def close(self):
        """關閉連線"""
        self.robot.stop_client()
        print("\n✓ 連線已關閉")


# ==================== 主程式 ====================

if __name__ == "__main__":
    print("\n" + "="*70)
    print("  TCP 自動校正系統")
    print("="*70)
    
    print("\n正在連接機器人...")
    HOST, PORT = "192.168.50.123", 5001
    calibrator = TCPAutoCalibration(HOST, PORT)
    
    try:
        print("\n請選擇:")
        print("  1. 自動模式 (先檢查，需要時校正)")
        print("  2. 強制完整校正")
        print("  3. 僅快速檢查")
        print("  4. 重置偏移量")
        
        choice = input("\n請輸入 (1-4) [預設:1]: ").strip()
        
        if choice == '2':
            result = calibrator.run_calibration(force_full=True)
        elif choice == '3':
            result = calibrator.quick_check()
        elif choice == '4':
            # 重置偏移量
            print(f"\n當前偏移量: ΔX={calibrator.tcp_offset['X']:+.3f}, "
                  f"ΔY={calibrator.tcp_offset['Y']:+.3f}, ΔZ={calibrator.tcp_offset['Z']:+.3f}")
            confirm = input("確定要重置為零? (輸入 yes): ").strip().lower()
            if confirm == 'yes':
                calibrator.tcp_offset = {'X': 0.0, 'Y': 0.0, 'Z': 0.0}
                calibrator.TCP_TARGET = calibrator.apply_offset()
                calibrator.save_offset(calibrator.tcp_offset)
                print("✓ 已重置")
        else:
            result = calibrator.run_calibration(force_full=False)
        
        print("\n" + "="*70)
        print("  流程結束")
        print("="*70)
        
    except KeyboardInterrupt:
        print("\n\n⚠️ 被使用者中斷")
    except Exception as e:
        print(f"\n✗ 錯誤: {e}")
        import traceback
        traceback.print_exc()
    finally:
        calibrator.close()