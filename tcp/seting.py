from pymycobot.elephantrobot import ElephantRobot
import time

def set_robot_orientation(host, port, rx, ry, rz, speed=30):
    """設定機器人姿態
    
    Args:
        host: 機器人 IP
        port: 機器人端口
        rx: RX 角度
        ry: RY 角度  
        rz: RZ 角度
        speed: 移動速度
    """
    print(f"連接到機器人 {host}:{port}...")
    robot = ElephantRobot(host, port)
    robot.start_client()
    
    try:
        # 取得當前座標
        print("讀取當前位置...")
        current = robot.get_coords()
        print(f"當前座標: {current}")
        
        # 設定新姿態
        target = [
            current[0],  # X
            current[1],  # Y
            current[2],  # Z
            rx,          # RX
            ry,          # RY
            rz           # RZ
        ]
        
        print(f"\n設定姿態到: RX={rx}°, RY={ry}°, RZ={rz}°")
        print(f"目標座標: {target}")
        
        confirm = input("\n確定要移動嗎？(y/n): ")
        if confirm.lower() != 'y':
            print("取消移動")
            return
        
        print("\n開始移動...")
        robot.write_coords(target, speed)
        
        # 等待移動完成
        while robot.check_running():
            print(".", end='', flush=True)
            time.sleep(0.5)
        
        print("\n✓ 移動完成!")
        
        # 確認位置
        actual = robot.get_coords()
        print(f"實際座標: {actual}")
        
    finally:
        robot.stop_client()
        print("連線已關閉")


if __name__ == "__main__":
    # 設定機器人到 RX=180, RY=0, RZ=0
    set_robot_orientation(
        host="192.168.50.123",
        port=5001,
        rx=90,
        ry=0,
        rz=90,
        speed=500
    )