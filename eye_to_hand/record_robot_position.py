from pymycobot.elephantrobot import ElephantRobot
import time
import json
from datetime import datetime

def record_current_position(host, port, save_name=None):
    """記錄當前機器人的關節角度和座標
    
    Args:
        host: 機器人 IP
        port: 機器人端口
        save_name: 儲存的名稱（可選）
    
    Returns:
        dict: 包含關節角度和座標的字典
    """
    robot = ElephantRobot(host, port)
    
    try:
        print(f"📡 連接機器人 {host}:{port}...")
        if not robot.start_client():
            print("❌ 連接失敗")
            return None
        print("✓ 已連接\n")
        
        # === 讀取關節角度 ===
        print("🔧 讀取關節角度...")
        angles = robot.get_angles()
        
        if angles == robot.invalid_coords():
            print("❌ 無法讀取關節角度")
            return None
        
        print("當前關節角度:")
        for i, angle in enumerate(angles, 1):
            print(f"  J{i} = {angle:8.3f}°")
        
        # === 讀取笛卡爾座標 ===
        print("\n📍 讀取笛卡爾座標...")
        coords = robot.get_coords()
        
        if coords == robot.invalid_coords():
            print("❌ 無法讀取座標")
            return None
        
        print("當前座標:")
        print(f"  X  = {coords[0]:8.3f} mm")
        print(f"  Y  = {coords[1]:8.3f} mm")
        print(f"  Z  = {coords[2]:8.3f} mm")
        print(f"  RX = {coords[3]:8.3f}°")
        print(f"  RY = {coords[4]:8.3f}°")
        print(f"  RZ = {coords[5]:8.3f}°")
        
        # === 建立資料字典 ===
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        position_data = {
            "name": save_name if save_name else f"position_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            "timestamp": timestamp,
            "joint_angles": {
                "J1": round(angles[0], 3),
                "J2": round(angles[1], 3),
                "J3": round(angles[2], 3),
                "J4": round(angles[3], 3),
                "J5": round(angles[4], 3),
                "J6": round(angles[5], 3)
            },
            "coordinates": {
                "X": round(coords[0], 3),
                "Y": round(coords[1], 3),
                "Z": round(coords[2], 3),
                "RX": round(coords[3], 3),
                "RY": round(coords[4], 3),
                "RZ": round(coords[5], 3)
            },
            "angles_list": [round(a, 3) for a in angles],
            "coords_list": [round(c, 3) for c in coords]
        }
        
        # === 儲存到檔案 ===
        filename = f"robot_position_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(position_data, f, indent=2, ensure_ascii=False)
        
        print(f"\n💾 已儲存到: {filename}")
        
        # === 顯示 Python 程式碼 ===
        print("\n" + "="*60)
        print("📋 複製此程式碼以回到此位置:")
        print("="*60)
        print(f"\n# {save_name if save_name else '記錄的位置'} - {timestamp}")
        print("# 使用關節角度移動 (推薦):")
        print(f"robot.write_angles({position_data['angles_list']}, speed=30)")
        print("\n# 或使用座標移動:")
        print(f"robot.write_coords({position_data['coords_list']}, speed=30)")
        print("\n" + "="*60)
        
        return position_data
        
    except Exception as e:
        print(f"\n❌ 錯誤: {e}")
        import traceback
        traceback.print_exc()
        return None
        
    finally:
        robot.stop_client()
        print("\n🔌 已斷線")


def load_and_move_to_position(host, port, json_file):
    """從 JSON 檔案載入並移動到指定位置
    
    Args:
        host: 機器人 IP
        port: 機器人端口
        json_file: JSON 檔案路徑
    """
    robot = ElephantRobot(host, port)
    
    try:
        # 讀取 JSON 檔案
        print(f"📂 讀取檔案: {json_file}")
        with open(json_file, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        print(f"✓ 載入位置: {data['name']}")
        print(f"  記錄時間: {data['timestamp']}\n")
        
        # 連接機器人
        print(f"📡 連接機器人...")
        if not robot.start_client():
            print("❌ 連接失敗")
            return False
        print("✓ 已連接\n")
        
        # 顯示目標位置
        print("🎯 目標位置:")
        angles = data['angles_list']
        for i, angle in enumerate(angles, 1):
            print(f"  J{i} = {angle:8.3f}°")
        
        # 確認移動
        confirm = input("\n確定要移動到此位置? (y/n): ")
        if confirm.lower() != 'y':
            print("🚫 取消移動")
            return False
        
        # 執行移動
        print("\n🤖 移動中...")
        robot.write_angles(angles, 30)
        
        # 等待完成
        while robot.check_running():
            print(".", end='', flush=True)
            time.sleep(0.5)
        
        print("\n✓ 移動完成!")
        
        # 驗證位置
        time.sleep(0.5)
        actual = robot.get_angles()
        print("\n📊 位置驗證:")
        for i in range(6):
            error = abs(actual[i] - angles[i])
            status = "✓" if error < 1.0 else "❌"
            print(f"  J{i+1}: 目標={angles[i]:7.2f}°, 實際={actual[i]:7.2f}°, 誤差={error:5.2f}° {status}")
        
        return True
        
    except FileNotFoundError:
        print(f"❌ 找不到檔案: {json_file}")
        return False
    except Exception as e:
        print(f"❌ 錯誤: {e}")
        return False
    finally:
        robot.stop_client()
        print("\n🔌 已斷線")


def record_multiple_positions(host, port):
    """記錄多個位置（互動式）
    
    Args:
        host: 機器人 IP
        port: 機器人端口
    """
    positions = []
    
    print("=" * 60)
    print("多點位記錄模式")
    print("=" * 60)
    print("將機器人移動到想要的位置，然後按指示記錄\n")
    
    while True:
        print("\n" + "-" * 60)
        input("按 Enter 記錄當前位置（或 Ctrl+C 結束）...")
        
        position_name = input("輸入位置名稱（可選，直接按 Enter 跳過）: ").strip()
        if not position_name:
            position_name = None
        
        data = record_current_position(host, port, position_name)
        
        if data:
            positions.append(data)
            print(f"\n✓ 已記錄 {len(positions)} 個位置")
        
        cont = input("\n繼續記錄下一個位置? (y/n): ")
        if cont.lower() != 'y':
            break
    
    # 儲存所有位置
    if positions:
        filename = f"robot_positions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(positions, f, indent=2, ensure_ascii=False)
        
        print(f"\n💾 所有位置已儲存到: {filename}")
        print(f"✓ 共記錄 {len(positions)} 個位置")


if __name__ == "__main__":
    HOST = "192.168.50.123"
    PORT = 5001
    
    print("=" * 60)
    print("機器人位置記錄程式")
    print("=" * 60)
    print("\n選擇功能:")
    print("1. 記錄當前位置")
    print("2. 記錄多個位置")
    print("3. 從檔案載入並移動")
    
    choice = input("\n請選擇 (1/2/3): ").strip()
    
    if choice == "1":
        # 單次記錄
        name = input("\n輸入位置名稱（可選）: ").strip()
        record_current_position(HOST, PORT, name if name else None)
        
    elif choice == "2":
        # 多次記錄
        try:
            record_multiple_positions(HOST, PORT)
        except KeyboardInterrupt:
            print("\n\n🛑 已停止記錄")
            
    elif choice == "3":
        # 載入並移動
        filename = input("\n輸入 JSON 檔案名稱: ").strip()
        load_and_move_to_position(HOST, PORT, filename)
        
    else:
        print("無效選擇")