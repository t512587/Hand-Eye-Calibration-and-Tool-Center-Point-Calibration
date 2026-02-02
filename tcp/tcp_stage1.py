from pymycobot.elephantrobot import ElephantRobot
import time
import numpy as np
from scipy.spatial.transform import Rotation
import json

class Stage1_ManualTCPCalculation:
    """第一階段：手動多姿態法計算TCP偏移量"""
    
    def __init__(self, host, port):
        self.robot = ElephantRobot(host, port)
        self.robot.start_client()
        
        # 雷射訊號接腳
        self.LASER_X_PIN = 0
        self.LASER_Y_PIN = 1
        
        # 儲存測量數據
        self.measurements = []
        self.tcp_offset = None
    
    def check_laser_status(self):
        """檢查兩個雷射的當前狀態"""
        x_blocked = self.robot.get_digital_in(self.LASER_X_PIN) == 1
        y_blocked = self.robot.get_digital_in(self.LASER_Y_PIN) == 1
        return x_blocked, y_blocked
    
    def wait_for_laser_trigger(self, timeout=60, stable_time=0.3):
        """等待雷射觸發並穩定"""
        print(f"\n📡 等待雷射觸發... (超時: {timeout}秒)")
        print("請手動調整機器人位置，讓夾爪尖端觸發雷射")
        
        start_time = time.time()
        trigger_start = None
        
        while time.time() - start_time < timeout:
            x_blocked, y_blocked = self.check_laser_status()
            
            # 即時顯示狀態
            x_status = "🔴 BLOCKED" if x_blocked else "🟢 CLEAR"
            y_status = "🔴 BLOCKED" if y_blocked else "🟢 CLEAR"
            print(f"\rLaser X: {x_status}  |  Laser Y: {y_status}", end='', flush=True)
            
            # 檢查是否觸發
            if x_blocked or y_blocked:
                if trigger_start is None:
                    trigger_start = time.time()
                    print(f"\n\n⚡ 偵測到觸發！等待穩定...")
                
                # 檢查是否穩定
                if time.time() - trigger_start >= stable_time:
                    print(f"\n✓ 觸發穩定")
                    time.sleep(0.2)
                    return True, x_blocked, y_blocked
            else:
                # 未觸發，重置計時
                if trigger_start is not None:
                    print(f"\n⚠️  觸發中斷，請重新調整")
                trigger_start = None
            
            time.sleep(0.05)
        
        print(f"\n\n✗ 超時：未偵測到穩定觸發")
        return False, False, False
    
    def record_measurement_point(self, name, x_blocked, y_blocked):
        """記錄測量點"""
        print(f"\n{'='*70}")
        print(f"記錄測量點: {name}")
        print(f"{'='*70}")
        
        # 記錄當前機器人狀態
        flange_coords = self.robot.get_coords()
        joint_angles = self.robot.get_angles()
        
        measurement = {
            'name': name,
            'flange_coords': flange_coords,
            'joint_angles': joint_angles,
            'laser_x_blocked': x_blocked,
            'laser_y_blocked': y_blocked,
            'timestamp': time.time()
        }
        
        # 顯示記錄的數據
        print(f"\n✓ 已記錄:")
        print(f"  法蘭位置: X={flange_coords[0]:.2f}, Y={flange_coords[1]:.2f}, Z={flange_coords[2]:.2f}")
        print(f"  法蘭姿態: RX={flange_coords[3]:.2f}, RY={flange_coords[4]:.2f}, RZ={flange_coords[5]:.2f}")
        print(f"  關節角度: {[f'{a:.2f}' for a in joint_angles]}")
        print(f"  Laser X: {'✓ 觸發' if x_blocked else '✗ 未觸發'}")
        print(f"  Laser Y: {'✓ 觸發' if y_blocked else '✗ 未觸發'}")
        
        self.measurements.append(measurement)
        return measurement
    
    def run_calibration(self):
        """執行多姿態校正流程"""
        print("\n" + "="*70)
        print("  第一階段：手動多姿態TCP偏移量計算")
        print("="*70)
        print("\n📋 目的: 計算夾爪尖端相對於法蘭中心的偏移量")
        print("\n流程:")
        print("  1. 你手動調整機器人到不同姿態")
        print("  2. 讓夾爪尖端觸發雷射（X或Y任一個）")
        print("  3. 程式自動偵測並記錄位置")
        print("  4. 至少需要 4 個不同姿態")
        
        input("\n按 Enter 開始...")
        
        # 定義測量步驟
        measurement_steps = [
            {
                'name': '姿態1 - 垂直向下 (0°)',
                'instruction': '夾爪垂直向下，觸發雷射後保持0.3秒'
            },
            {
                'name': '姿態2 - 旋轉90°',
                'instruction': '保持垂直，末端法蘭旋轉90度（J6+90°）'
            },
            {
                'name': '姿態3 - 旋轉180°',
                'instruction': '保持垂直，末端法蘭旋轉180度（J6+180°）'
            },
            {
                'name': '姿態4 - 傾斜45°',
                'instruction': '夾爪傾斜約45度（J5±45°）'
            },
            {
                'name': '姿態5 - 側面接近',
                'instruction': '從側面接近雷射，改變接近角度'
            }
        ]
        
        # 執行每個測量步驟
        for i, step in enumerate(measurement_steps, 1):
            print(f"\n{'='*70}")
            print(f"  測量 {i}/{len(measurement_steps)}: {step['name']}")
            print(f"{'='*70}")
            print(f"\n指示: {step['instruction']}")
            
            input("\n調整好後按 Enter，開始偵測...")
            
            # 等待雷射觸發
            triggered, x_blocked, y_blocked = self.wait_for_laser_trigger()
            
            if triggered:
                # 記錄測量點
                self.record_measurement_point(step['name'], x_blocked, y_blocked)
            else:
                retry = input("\n未偵測到觸發，是否重試此姿態？(y/n): ")
                if retry.lower() == 'y':
                    # 重做這一步（透過修改迴圈索引不太實際，這裡採用遞迴）
                    print("\n重新開始此姿態...")
                    input("按 Enter 繼續...")
                    
                    triggered, x_blocked, y_blocked = self.wait_for_laser_trigger()
                    if triggered:
                        self.record_measurement_point(step['name'], x_blocked, y_blocked)
                    else:
                        print(f"\n⚠️  跳過此姿態")
                        continue
        
        # 顯示所有測量結果
        self.display_all_measurements()
        
        # 計算TCP偏移量
        if len(self.measurements) >= 4:
            print(f"\n✓ 共收集 {len(self.measurements)} 個有效測量點")
            self.tcp_offset = self.calculate_tcp_offset()
            
            if self.tcp_offset:
                # 套用TCP偏移量
                applied = self.apply_tcp_offset(self.tcp_offset)
                
                if applied:
                    # 儲存結果
                    self.save_results()
                    
                    print("\n" + "="*70)
                    print("  ✓✓✓ 第一階段完成！ ✓✓✓")
                    print("="*70)
                    print("\nTCP偏移量已計算並套用到機器人")
                    print("\n接下來可以:")
                    print("  1. 執行 stage2_manual_precision.py 進行精確校正")
                    print("  2. 直接使用 robot.get_tool_coords() 和 robot.write_tool_coords()")
                    
                    return self.tcp_offset
        else:
            print(f"\n✗ 測量點不足（需要至少4個，目前{len(self.measurements)}個）")
            return None
    
    def display_all_measurements(self):
        """顯示所有測量結果"""
        print("\n" + "="*70)
        print("  所有測量點摘要")
        print("="*70)
        
        for i, m in enumerate(self.measurements, 1):
            print(f"\n測量點 {i}: {m['name']}")
            print(f"  法蘭位置: X={m['flange_coords'][0]:.2f}, "
                  f"Y={m['flange_coords'][1]:.2f}, "
                  f"Z={m['flange_coords'][2]:.2f}")
            print(f"  法蘭姿態: RX={m['flange_coords'][3]:.2f}, "
                  f"RY={m['flange_coords'][4]:.2f}, "
                  f"RZ={m['flange_coords'][5]:.2f}")
            print(f"  雷射狀態: X={'✓' if m['laser_x_blocked'] else '✗'}, "
                  f"Y={'✓' if m['laser_y_blocked'] else '✗'}")
    
    def calculate_tcp_offset(self):
        """從測量數據計算TCP偏移量"""
        print("\n" + "="*70)
        print("  計算TCP偏移量")
        print("="*70)
        
        n = len(self.measurements)
        if n < 4:
            raise ValueError("TCP校正至少需要4個姿態")
        
        # 建立齊次轉換矩陣
        T_flanges = []
        for m in self.measurements:
            x, y, z, rx, ry, rz = m['flange_coords']
            R = Rotation.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [x, y, z]
            T_flanges.append(T)
        
        positions = np.array([T[:3, 3] for T in T_flanges])
        rotations = np.array([T[:3, :3] for T in T_flanges])
        
        # 建立線性方程組
        A = []
        b = []
        
        for i in range(n):
            for j in range(i + 1, n):
                Rij = rotations[i] - rotations[j]
                pij = positions[j] - positions[i]
                
                for k in range(3):
                    A.append(Rij[k, :])
                    b.append(pij[k])
        
        A = np.array(A)
        b = np.array(b)
        
        # 求解
        tcp_offset, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        
        # 診斷資訊
        print(f"\n線性系統診斷:")
        print(f"  A矩陣形狀: {A.shape}")
        print(f"  矩陣秩 (rank): {rank}")
        
        if rank < 3:
            print("  ⚠️ 警告：姿態變化不足，TCP解可能不可靠")
        
        if len(residuals) > 0:
            print(f"  最小平方殘差: {residuals[0]:.4f}")
        else:
            print(f"  最小平方殘差: N/A")
        
        # 驗證：所有TCP是否映射到同一個雷射點
        laser_points = []
        for T in T_flanges:
            P_tcp_h = np.append(tcp_offset, 1)
            P_laser = T @ P_tcp_h
            laser_points.append(P_laser[:3])
        
        laser_points = np.array(laser_points)
        mean_laser = np.mean(laser_points, axis=0)
        std_laser = np.std(laser_points, axis=0)
        max_err = np.max(np.linalg.norm(laser_points - mean_laser, axis=1))
        
        print(f"\n雷射觸發點一致性檢查:")
        print(f"  平均位置: {mean_laser}")
        print(f"  標準差:   {std_laser}")
        print(f"  最大誤差: {max_err:.3f} mm")
        
        if max_err > 5.0:
            print(f"  ⚠️ 警告：最大誤差較大，建議重新測量")
        
        # 整理結果
        result = {
            'X': round(float(tcp_offset[0]), 2),
            'Y': round(float(tcp_offset[1]), 2),
            'Z': round(float(tcp_offset[2]), 2),
            'RX': 0.0,
            'RY': 0.0,
            'RZ': 0.0
        }
        
        print("\n" + "="*70)
        print("  TCP偏移量（相對於法蘭中心）")
        print("="*70)
        print(f"  X:  {result['X']:>8.2f} mm")
        print(f"  Y:  {result['Y']:>8.2f} mm")
        print(f"  Z:  {result['Z']:>8.2f} mm")
        print(f"  RX: {result['RX']:>8.2f} °")
        print(f"  RY: {result['RY']:>8.2f} °")
        print(f"  RZ: {result['RZ']:>8.2f} °")
        print("="*70)
        
        return result
    
    def apply_tcp_offset(self, offset):
        """套用TCP偏移量到機器人"""
        print("\n" + "="*70)
        print("  套用TCP偏移量")
        print("="*70)
        
        tool_reference = np.array([
            offset['X'],
            offset['Y'],
            offset['Z'],
            offset['RX'],
            offset['RY'],
            offset['RZ']
        ], dtype=float)
        
        confirm = input(f"\n確定要套用此偏移量到機器人嗎？(y/n): ")
        if confirm.lower() != 'y':
            print("取消套用")
            return False
        
        # 套用到機器人
        self.robot.tool_matrix = self.robot.set_tool_reference(tool_reference)
        print(f"\n✓ TCP偏移量已套用: {tool_reference}")
        
        time.sleep(0.5)
        
        # 驗證
        print(f"\n驗證:")
        flange_coords = self.robot.get_coords()
        tool_coords = self.robot.get_tool_coords()
        
        print(f"  法蘭座標: {[f'{c:.2f}' for c in flange_coords]}")
        print(f"  工具座標: {[f'{c:.2f}' for c in tool_coords]}")
        
        # 計算實際偏移距離
        flange_pos = np.array(flange_coords[:3])
        tool_pos = np.array(tool_coords[:3])
        offset_distance = np.linalg.norm(tool_pos - flange_pos)
        expected_distance = np.linalg.norm([offset['X'], offset['Y'], offset['Z']])
        
        print(f"\n  期望偏移距離: {expected_distance:.2f} mm")
        print(f"  實際偏移距離: {offset_distance:.2f} mm")
        print(f"  誤差: {abs(offset_distance - expected_distance):.2f} mm")
        
        if abs(offset_distance - expected_distance) < 1.0:
            print("  ✓ TCP偏移量驗證成功！")
            return True
        else:
            print("  ⚠️ 偏移距離不符，請檢查")
            return False
    
    def save_results(self):
        """儲存校正結果"""
        # 儲存為JSON
        results = {
            'tcp_offset': self.tcp_offset,
            'measurements': self.measurements,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        
        filename = 'tcp_stage1_results.json'
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        
        print(f"\n✓ 結果已儲存至: {filename}")
        
        # 同時儲存為可讀的文字檔
        txt_filename = 'tcp_stage1_results.txt'
        with open(txt_filename, 'w', encoding='utf-8') as f:
            f.write("="*70 + "\n")
            f.write("第一階段：TCP偏移量計算結果\n")
            f.write("="*70 + "\n")
            f.write(f"校正時間: {results['timestamp']}\n")
            f.write(f"測量點數量: {len(self.measurements)}\n\n")
            
            f.write("TCP偏移量（相對於法蘭中心）:\n")
            f.write(f"  X:  {self.tcp_offset['X']:>8.2f} mm\n")
            f.write(f"  Y:  {self.tcp_offset['Y']:>8.2f} mm\n")
            f.write(f"  Z:  {self.tcp_offset['Z']:>8.2f} mm\n")
            f.write(f"  RX: {self.tcp_offset['RX']:>8.2f} °\n")
            f.write(f"  RY: {self.tcp_offset['RY']:>8.2f} °\n")
            f.write(f"  RZ: {self.tcp_offset['RZ']:>8.2f} °\n\n")
            
            f.write("="*70 + "\n")
            f.write("測量點詳細資料\n")
            f.write("="*70 + "\n\n")
            
            for i, m in enumerate(self.measurements, 1):
                f.write(f"測量點 {i}: {m['name']}\n")
                f.write(f"  法蘭座標: {m['flange_coords']}\n")
                f.write(f"  關節角度: {m['joint_angles']}\n")
                f.write(f"  雷射狀態: X={m['laser_x_blocked']}, Y={m['laser_y_blocked']}\n\n")
        
        print(f"✓ 詳細資料已儲存至: {txt_filename}")
    
    def close(self):
        """關閉連線"""
        self.robot.stop_client()


# ==================== 主程式 ====================

if __name__ == "__main__":
    print("="*70)
    print("  TCP校正系統 - 第一階段")
    print("  手動多姿態法計算TCP偏移量")
    print("="*70)
    
    print("\n正在連接機器人...")
    calibrator = Stage1_ManualTCPCalculation("192.168.50.123", 5001)
    
    try:
        # 執行校正
        tcp_offset = calibrator.run_calibration()
        
        if tcp_offset:
            print("\n" + "="*70)
            print("  第一階段完成！")
            print("="*70)
            print("\n下一步：執行 stage2_manual_precision.py")
            print("進行手動精確校正（單軸逼近驗證）")
        else:
            print("\n" + "="*70)
            print("  第一階段失敗或被中斷")
            print("="*70)
    
    except KeyboardInterrupt:
        print("\n\n校正被使用者中斷")
    except Exception as e:
        print(f"\n錯誤: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n正在關閉連線...")
        calibrator.close()