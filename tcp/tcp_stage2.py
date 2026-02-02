from pymycobot.elephantrobot import ElephantRobot
import time
import json
import os

class Stage2_ManualPrecisionCalibration:
    """第二階段：手動精確校正（單軸逼近驗證）
    
    注意：此階段記錄的雷射觸發點位置使用「法蘭座標系」而非工具座標系
    """
    
    def __init__(self, host, port):
        self.robot = ElephantRobot(host, port)
        self.robot.start_client()
        
        # 雷射訊號接腳
        self.LASER_X_PIN = 0
        self.LASER_Y_PIN = 1
        
        # 儲存校正結果
        self.calibration_results = {
            'x_axis': {},
            'y_axis': {},
            'z_axis': {}
        }
        
        # 載入第一階段結果
        self.tcp_offset = None
        self.load_stage1_results()
    
    def load_stage1_results(self):
        """載入第一階段的TCP偏移量"""
        filename = 'tcp_stage1_results.json'
        
        if not os.path.exists(filename):
            print(f"\n⚠️ 警告：找不到第一階段結果檔案 '{filename}'")
            print("請先執行 stage1_tcp_offset.py")
            return False
        
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.tcp_offset = data['tcp_offset']
            
            print(f"\n✓ 已載入第一階段結果:")
            print(f"  TCP偏移量: X={self.tcp_offset['X']:.2f}, "
                  f"Y={self.tcp_offset['Y']:.2f}, "
                  f"Z={self.tcp_offset['Z']:.2f}")
            
            # 套用TCP偏移量到機器人
            import numpy as np
            tool_reference = np.array([
                self.tcp_offset['X'],
                self.tcp_offset['Y'],
                self.tcp_offset['Z'],
                self.tcp_offset['RX'],
                self.tcp_offset['RY'],
                self.tcp_offset['RZ']
            ], dtype=float)
            
            self.robot.tool_matrix = self.robot.set_tool_reference(tool_reference)
            print(f"  ✓ TCP偏移量已套用到機器人")
            
            return True
            
        except Exception as e:
            print(f"\n✗ 載入第一階段結果失敗: {e}")
            return False
    
    def check_laser(self, axis):
        """檢查指定軸的雷射狀態"""
        if axis.lower() == 'x':
            return self.robot.get_digital_in(self.LASER_X_PIN) == 1
        elif axis.lower() == 'y':
            return self.robot.get_digital_in(self.LASER_Y_PIN) == 1
        else:
            raise ValueError("axis 必須是 'x' 或 'y'")
    
    def check_both_lasers(self):
        """檢查兩個雷射狀態"""
        x_blocked = self.check_laser('x')
        y_blocked = self.check_laser('y')
        return x_blocked, y_blocked
    
    def display_laser_status(self):
        """即時顯示雷射狀態"""
        x_blocked, y_blocked = self.check_both_lasers()
        x_status = "🔴 BLOCKED" if x_blocked else "🟢 CLEAR"
        y_status = "🔴 BLOCKED" if y_blocked else "🟢 CLEAR"
        
        print(f"\rLaser X: {x_status}  |  Laser Y: {y_status}", end='', flush=True)
        return x_blocked, y_blocked
    
    def wait_for_stable_position(self, timeout=30):
        """等待位置穩定（用於手動調整後）"""
        print("\n等待位置穩定...")
        time.sleep(1.0)  # 給予1秒緩衝時間
        print("✓ 位置已穩定")
    
    def manual_single_axis_calibration(self, axis_name):
        """手動單軸校正（X或Y軸）- 自動偵測觸發
        
        Args:
            axis_name: 'X' 或 'Y'
        
        Returns:
            dict: {'pos1': float, 'pos2': float, 'center': float}
        """
        axis_lower = axis_name.lower()
        axis_index = {'x': 0, 'y': 1}[axis_lower]
        
        print("\n" + "="*70)
        print(f"  {axis_name}軸手動校正 - 雙向逼近法（自動偵測）")
        print("="*70)
        
        print(f"\n原理:")
        print(f"  1. 從一側接近，讓TCP觸發 Laser {axis_name}")
        print(f"  2. 程式自動偵測觸發並記錄位置 {axis_name}1")
        print(f"  3. 從另一側接近，再次觸發")
        print(f"  4. 程式自動記錄位置 {axis_name}2")
        print(f"  5. 計算中心 = ({axis_name}1 + {axis_name}2) / 2")
        
        # === 第一次測量：從負方向接近 ===
        print(f"\n--- 第1次測量：從負方向接近 ---")
        print(f"請手動移動機器人，讓TCP從 {axis_name}軸負方向慢慢接近雷射")
        print(f"⚠️ 當觸發後，請保持位置不動2秒，等待記錄完成")
        
        input("\n準備好後按 Enter 開始監控...")
        
        print("\n開始監控雷射狀態...")
        print("等待觸發...")
        
        # 自動偵測觸發
        triggered = False
        trigger_count = 0
        required_triggers = 5  # 需要連續5次觸發才確認（約0.25秒）
        
        while not triggered:
            try:
                laser_blocked = self.check_laser(axis_lower)
                status = "🔴 BLOCKED" if laser_blocked else "🟢 CLEAR"
                print(f"\rLaser {axis_name}: {status}  ", end='', flush=True)
                
                if laser_blocked:
                    trigger_count += 1
                    if trigger_count >= required_triggers:
                        print(f"\n\n✓ 偵測到穩定觸發！請保持位置不動...")
                        triggered = True
                else:
                    if trigger_count > 0:
                        # 曾經觸發但又消失了，重置
                        trigger_count = 0
                
                time.sleep(0.05)
            except Exception as e:
                print(f"\n錯誤: {e}")
                print("請稍後重試...")
                time.sleep(0.5)
        
        # 等待位置穩定
        print("記錄位置中...")
        time.sleep(1.0)
        
        # 記錄第一個位置（使用法蘭座標系，而非工具座標系）
        flange_coords_1 = self.robot.get_coords()
        pos1 = flange_coords_1[axis_index]
        
        print(f"\n✓ 已記錄第1個觸發位置:")
        print(f"  {axis_name}1 = {pos1:.2f} mm")
        print(f"  完整法蘭座標: {[f'{c:.2f}' for c in flange_coords_1[:3]]}")
        
        # === 第二次測量：從正方向接近 ===
        print(f"\n--- 第2次測量：從正方向接近 ---")
        print(f"請手動移動機器人，讓TCP從 {axis_name}軸正方向慢慢接近雷射")
        print(f"⚠️ 當觸發後，請保持位置不動2秒，等待記錄完成")
        
        input("\n準備好後按 Enter 開始監控...")
        
        print("\n開始監控雷射狀態...")
        print("等待觸發...")
        
        # 自動偵測觸發（第二次）
        triggered = False
        trigger_count = 0
        
        while not triggered:
            try:
                laser_blocked = self.check_laser(axis_lower)
                status = "🔴 BLOCKED" if laser_blocked else "🟢 CLEAR"
                print(f"\rLaser {axis_name}: {status}  ", end='', flush=True)
                
                if laser_blocked:
                    trigger_count += 1
                    if trigger_count >= required_triggers:
                        print(f"\n\n✓ 偵測到穩定觸發！請保持位置不動...")
                        triggered = True
                else:
                    if trigger_count > 0:
                        trigger_count = 0
                
                time.sleep(0.05)
            except Exception as e:
                print(f"\n錯誤: {e}")
                print("請稍後重試...")
                time.sleep(0.5)
        
        # 等待位置穩定
        print("記錄位置中...")
        time.sleep(1.0)
        
        # 記錄第二個位置（使用法蘭座標系，而非工具座標系）
        flange_coords_2 = self.robot.get_coords()
        pos2 = flange_coords_2[axis_index]
        
        print(f"\n✓ 已記錄第2個觸發位置:")
        print(f"  {axis_name}2 = {pos2:.2f} mm")
        print(f"  完整法蘭座標: {[f'{c:.2f}' for c in flange_coords_2[:3]]}")
        
        # 計算中心
        center = (pos1 + pos2) / 2
        
        print(f"\n" + "="*70)
        print(f"  {axis_name}軸校正結果")
        print(f"="*70)
        print(f"  {axis_name}1 (負向): {pos1:>8.2f} mm")
        print(f"  {axis_name}2 (正向): {pos2:>8.2f} mm")
        print(f"  中心位置:    {center:>8.2f} mm")
        print(f"  檢測範圍:    {abs(pos2-pos1):>8.2f} mm")
        print(f"="*70)
        
        result = {
            'pos1': pos1,
            'pos2': pos2,
            'center': center,
            'range': abs(pos2 - pos1)
        }
        
        self.calibration_results[f'{axis_lower}_axis'] = result
        
        return result
    
    def manual_z_axis_calibration(self, x_center, y_center):
        """手動Z軸校正（同時觸發法）
        
        Args:
            x_center: X軸中心位置
            y_center: Y軸中心位置
        
        Returns:
            dict: {'z_trigger': float}
        """
        print("\n" + "="*70)
        print("  Z軸手動校正 - 同時觸發法")
        print("="*70)
        
        print(f"\n原理:")
        print(f"  1. 將TCP移動到XY中心位置上方")
        print(f"     (X={x_center:.2f}, Y={y_center:.2f})")
        print(f"  2. 慢慢下降")
        print(f"  3. 當XY雷射同時觸發時，記錄Z高度")
        
        print(f"\n提示:")
        print(f"  - 先移動到XY中心上方（可以手動或用程式）")
        print(f"  - 然後慢慢下降")
        print(f"  - 看到兩個雷射都變紅色時停止")
        
        # 詢問是否自動移動到XY中心上方
        auto_move = input(f"\n是否自動移動到XY中心上方？(y/n): ")
        
        if auto_move.lower() == 'y':
            current_tcp = self.robot.get_tool_coords()
            target_coords = list(current_tcp)  # 轉換為列表
            target_coords[0] = x_center
            target_coords[1] = y_center
            target_coords[2] = current_tcp[2] + 50  # 上升50mm作為安全高度
            
            print(f"\n移動到: X={target_coords[0]:.2f}, Y={target_coords[1]:.2f}, Z={target_coords[2]:.2f}")
            
            confirm = input("確定移動？(y/n): ")
            if confirm.lower() == 'y':
                self.robot.write_tool_coords(target_coords, 30)
                
                print("移動中...", end='', flush=True)
                while self.robot.check_running():
                    print(".", end='', flush=True)
                    time.sleep(0.5)
                print(" 完成!")
                
                time.sleep(0.5)
        
        # 開始監控
        print(f"\n請手動慢慢下降TCP")
        print(f"程式會自動偵測XY雷射同時觸發並記錄Z高度")
        print(f"⚠️ 觸發後請保持位置不動2秒")
        
        input("\n按 Enter 開始監控...")
        
        print("\n監控雷射狀態...")
        print("等待 XY 同時觸發...")
        
        # 自動偵測同時觸發
        triggered = False
        trigger_count = 0
        required_triggers = 5  # 需要連續5次同時觸發
        
        while not triggered:
            try:
                x_blocked, y_blocked = self.display_laser_status()
                
                if x_blocked and y_blocked:
                    trigger_count += 1
                    if trigger_count >= required_triggers:
                        print(f"\n\n✓ 偵測到XY同時觸發！請保持位置不動...")
                        triggered = True
                else:
                    if trigger_count > 0:
                        trigger_count = 0
                
                time.sleep(0.05)
            except Exception as e:
                print(f"\n錯誤: {e}")
                print("請稍後重試...")
                time.sleep(0.5)
        
        # 等待位置穩定
        print("記錄位置中...")
        time.sleep(1.0)
        
        # 記錄Z位置（使用法蘭座標系，而非工具座標系）
        flange_coords = self.robot.get_coords()
        z_trigger = flange_coords[2]
        
        print(f"\n\n✓ 已記錄Z軸觸發位置:")
        print(f"  Z = {z_trigger:.2f} mm")
        print(f"  完整法蘭座標: {[f'{c:.2f}' for c in flange_coords[:3]]}")
        
        # 驗證：是否真的兩個雷射都觸發了
        x_blocked, y_blocked = self.check_both_lasers()
        if x_blocked and y_blocked:
            print(f"  ✓ 驗證通過：XY雷射確實同時觸發")
        else:
            print(f"  ⚠️ 警告：當前XY雷射未同時觸發")
            print(f"     Laser X: {'觸發' if x_blocked else '未觸發'}")
            print(f"     Laser Y: {'觸發' if y_blocked else '未觸發'}")
        
        print(f"\n" + "="*70)
        print(f"  Z軸校正結果")
        print(f"="*70)
        print(f"  Z軸觸發高度: {z_trigger:>8.2f} mm")
        print(f"="*70)
        
        result = {
            'z_trigger': z_trigger,
            'both_triggered': x_blocked and y_blocked
        }
        
        self.calibration_results['z_axis'] = result
        
        return result
    
    def run_calibration(self):
        """執行完整的手動精確校正流程"""
        print("\n" + "="*70)
        print("  第二階段：手動精確校正")
        print("="*70)
        
        print("\n📋 目的: 使用單軸逼近法精確測量TCP位置")
        
        print("\n流程:")
        print("  1. X軸：從左右兩側手動接近 → 計算中心")
        print("  2. Y軸：從前後兩側手動接近 → 計算中心")
        print("  3. Z軸：下降直到XY同時觸發 → 記錄高度")
        
        print("\n注意:")
        print("  - 這個階段需要你手動慢慢調整機器人")
        print("  - 程式會監控雷射狀態並記錄觸發位置")
        print("  - 移動要慢、要細心，確保精確觸發")
        
        if not self.tcp_offset:
            print("\n✗ 未找到第一階段結果，無法繼續")
            print("   請先執行 stage1_tcp_offset.py")
            return None
        
        input("\n按 Enter 開始...")
        
        # X軸校正
        x_result = self.manual_single_axis_calibration('X')
        
        if not x_result:
            print("\n✗ X軸校正失敗")
            return None
        
        # Y軸校正
        y_result = self.manual_single_axis_calibration('Y')
        
        if not y_result:
            print("\n✗ Y軸校正失敗")
            return None
        
        # Z軸校正
        z_result = self.manual_z_axis_calibration(
            x_result['center'],
            y_result['center']
        )
        
        if not z_result:
            print("\n✗ Z軸校正失敗")
            return None
        
        # 顯示最終結果
        self.display_final_results()
        
        # 儲存結果
        self.save_results()
        
        return self.calibration_results
    
    def display_final_results(self):
        """顯示最終校正結果"""
        print("\n" + "="*70)
        print("  ✓✓✓ 第二階段完成！ ✓✓✓")
        print("="*70)
        
        x_center = self.calibration_results['x_axis']['center']
        y_center = self.calibration_results['y_axis']['center']
        z_center = self.calibration_results['z_axis']['z_trigger']
        
        print(f"\n精確雷射觸發點位置（法蘭座標系）:")
        print(f"  X = {x_center:>8.2f} mm")
        print(f"  Y = {y_center:>8.2f} mm")
        print(f"  Z = {z_center:>8.2f} mm")
        
        print(f"\nTCP偏移量（已套用）:")
        print(f"  X = {self.tcp_offset['X']:>8.2f} mm")
        print(f"  Y = {self.tcp_offset['Y']:>8.2f} mm")
        print(f"  Z = {self.tcp_offset['Z']:>8.2f} mm")
        
        print(f"\n檢測範圍:")
        print(f"  X軸範圍: {self.calibration_results['x_axis']['range']:>8.2f} mm")
        print(f"  Y軸範圍: {self.calibration_results['y_axis']['range']:>8.2f} mm")
        
        print("="*70)
    
    def save_results(self):
        """儲存校正結果"""
        results = {
            'tcp_offset': self.tcp_offset,
            'precise_position': {
                'x': self.calibration_results['x_axis']['center'],
                'y': self.calibration_results['y_axis']['center'],
                'z': self.calibration_results['z_axis']['z_trigger']
            },
            'calibration_details': self.calibration_results,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S')
        }
        
        # 儲存JSON
        filename = 'tcp_stage2_results.json'
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        
        print(f"\n✓ 結果已儲存至: {filename}")
        
        # 儲存文字檔
        txt_filename = 'tcp_stage2_results.txt'
        with open(txt_filename, 'w', encoding='utf-8') as f:
            f.write("="*70 + "\n")
            f.write("第二階段：手動精確校正結果\n")
            f.write("="*70 + "\n")
            f.write(f"校正時間: {results['timestamp']}\n\n")
            
            f.write("精確雷射觸發點位置（法蘭座標系）:\n")
            f.write(f"  X = {results['precise_position']['x']:>8.2f} mm\n")
            f.write(f"  Y = {results['precise_position']['y']:>8.2f} mm\n")
            f.write(f"  Z = {results['precise_position']['z']:>8.2f} mm\n\n")
            
            f.write("TCP偏移量（已套用）:\n")
            f.write(f"  X = {self.tcp_offset['X']:>8.2f} mm\n")
            f.write(f"  Y = {self.tcp_offset['Y']:>8.2f} mm\n")
            f.write(f"  Z = {self.tcp_offset['Z']:>8.2f} mm\n\n")
            
            f.write("="*70 + "\n")
            f.write("詳細校正數據\n")
            f.write("="*70 + "\n\n")
            
            f.write("X軸:\n")
            f.write(f"  X1 (負向): {self.calibration_results['x_axis']['pos1']:.2f} mm\n")
            f.write(f"  X2 (正向): {self.calibration_results['x_axis']['pos2']:.2f} mm\n")
            f.write(f"  中心:      {self.calibration_results['x_axis']['center']:.2f} mm\n")
            f.write(f"  範圍:      {self.calibration_results['x_axis']['range']:.2f} mm\n\n")
            
            f.write("Y軸:\n")
            f.write(f"  Y1 (負向): {self.calibration_results['y_axis']['pos1']:.2f} mm\n")
            f.write(f"  Y2 (正向): {self.calibration_results['y_axis']['pos2']:.2f} mm\n")
            f.write(f"  中心:      {self.calibration_results['y_axis']['center']:.2f} mm\n")
            f.write(f"  範圍:      {self.calibration_results['y_axis']['range']:.2f} mm\n\n")
            
            f.write("Z軸:\n")
            f.write(f"  觸發高度:  {self.calibration_results['z_axis']['z_trigger']:.2f} mm\n")
            f.write(f"  同時觸發:  {'是' if self.calibration_results['z_axis']['both_triggered'] else '否'}\n")
        
        print(f"✓ 詳細資料已儲存至: {txt_filename}")
    
    def close(self):
        """關閉連線"""
        self.robot.stop_client()


# ==================== 主程式 ====================

if __name__ == "__main__":
    print("="*70)
    print("  TCP校正系統 - 第二階段")
    print("  手動精確校正（單軸逼近驗證）")
    print("="*70)
    
    print("\n正在連接機器人...")
    calibrator = Stage2_ManualPrecisionCalibration("192.168.50.123", 5001)
    
    try:
        # 執行校正
        results = calibrator.run_calibration()
        
        if results:
            print("\n" + "="*70)
            print("  ✓✓✓ TCP完整校正流程結束！ ✓✓✓")
            print("="*70)
            print("\n現在可以:")
            print("  - 使用 robot.get_tool_coords() 獲取TCP座標")
            print("  - 使用 robot.write_tool_coords() 用TCP座標移動")
            print("  - TCP偏移量已正確套用到機器人")
        else:
            print("\n" + "="*70)
            print("  第二階段失敗或被中斷")
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