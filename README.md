# 🤖 Elephant Robotics 手眼標定與 TCP 校正

[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Robot](https://img.shields.io/badge/Robot-Elephant_Robotics-orange.svg)](https://www.elephantrobotics.com/)
[![Camera](https://img.shields.io/badge/Camera-Intel_RealSense-00C9FF.svg)](https://www.intelrealsense.com/)

本專案提供一套為 **大象機器人 (Elephant Robotics)** 與 **Intel RealSense D400 系列相機** 設計的完整解決方案，包含：
- **TCP 校正（Tool Center Point Calibration）** - 精確校正工具中心點位置
- **手眼標定（Hand-Eye Calibration）** - 支援 Eye-in-Hand 與 Eye-to-Hand 兩種配置

---

## 📖 目錄

- [系統架構](#-系統架構)
- [硬體配置與環境](#-硬體配置與環境)
- [安裝指南](#-安裝指南)
- [專案結構](#-專案結構)
- [TCP 校正流程](#-tcp-校正流程)
- [手眼標定流程](#-手眼標定流程)
- [進階設定](#-進階設定)
- [授權](#-授權)

---

## 🏗 系統架構

### 1. TCP 校正
精確定位機械臂末端工具的實際位置，是手眼標定的**前置步驟**。

```
法蘭中心 → TCP 偏移量 → 工具尖端實際位置
```

### 2. 手眼標定

#### Eye-in-Hand（眼在手）
相機固定於機械臂末端，隨手臂移動。
```
機械臂末端 → 相機 → 觀測目標（棋盤格標定板）
```

#### Eye-to-Hand（眼不在手）
相機固定於外部位置，觀測機械臂末端的標記物。
```
外部相機 → 觀測 → 機械臂末端（ArUco Marker）
```

---

## 🛠 硬體配置與環境

### 硬體需求

| 組件 | 規格 | 用途 |
|------|------|------|
| **機械臂** | Elephant Robotics 系列 | myCobotpro630 等 |
| **相機** | Intel RealSense D415/D435/D455 | 深度相機 |
| **TCP 校正設備** | 十字雷射對位器 | TCP 校正 |
| **手眼標定標記物** | ArUco Marker / 棋盤格 (9×6) | 手眼標定 |
| **連線方式** | TCP/IP | IP: `192.168.50.123`, Port: `5001` |

### 軟體環境

```bash
pip install numpy opencv-contrib-python pyrealsense2 pymycobot scipy matplotlib
```

---

## 📁 專案結構

```
Hand-Eye-Calibration/
├── tcp/                            # TCP 校正
│   ├── tcp_stage1.py               # 階段一：計算夾爪偏移量
│   ├── tcp_stage2.py               # 階段二：定位雷射中心點
│   ├── tcp_auto.py                 # 自動化校正
│   ├── seting.py                   # 姿態設定工具
│   └── tcp_offset.json             # 偏移量記錄
│
├── eye_in_hand/                    # 眼在手模式
│   ├── record.py                   # 記錄標定數據
│   ├── compute_transform.py        # 計算轉換矩陣
│   ├── auto.py                     # 自動化標定
│   └── handeye_records/            # 數據儲存目錄
│
└── eye_to_hand/                    # 眼不在手模式
    ├── record.py                   # 記錄標定數據
    ├── compute_transform.py        # 計算轉換矩陣
    ├── auto.py                     # 自動化標定
    └── aruco_records/              # 數據儲存目錄
```

---

## 🎯 TCP 校正流程




### 兩階段手動校正

**階段一 - 計算偏移量：**
```bash
python tcp_stage1.py
```
- 手動移動至 5 個不同姿態
- 每個姿態觸發雷射並記錄
- 自動計算 TCP 偏移量

**階段二 - 精確定位：**
```bash
python tcp_stage2.py
```
- 基於階段一結果
- X/Y/Z 軸逐一精確定位
- 輸出最終 TCP 位置

---

### 自動化校正

**設備準備：**
- 將十字雷射對位器固定於工作台
- 連接雷射訊號至機械臂 I/O（Pin 0, Pin 1）

**執行校正：**
```bash
cd tcp
python tcp_auto.py
```

**自動流程：**
1. 載入上次校正結果（如有）
2. 移動至雷射上方
3. 執行 Z 軸搜尋
4. 圓形掃描觸發雷射
5. 計算並保存 TCP 偏移量

**輸出檔案：** `tcp_offset.json`

```json
{
  "offset": {
    "X": 0.654,
    "Y": -0.327,
    "Z": 0.145
  },
  "calibrated_tcp": {
    "X": 214.875,
    "Y": -61.055,
    "Z": 224.392
  }
}
```

## 📐 手眼標定流程

### Eye-in-Hand 模式

**步驟 1：準備棋盤格標定板**
- 規格：9×6 內角點，方格 25mm

**步驟 2：記錄標定數據**
```bash
cd eye_in_hand
python record.py
```
- 按 `S` 記錄姿態
- 按 `V` 查看記錄
- 按 `Q` 退出儲存

**步驟 3：計算轉換矩陣**
```bash
python compute_transform.py
```

**步驟 4：即時測試**
- 即時檢測棋盤格
- 轉換為機械臂座標
- 按 `C` 輸出座標

---

### Eye-to-Hand 模式

**步驟 1：準備 ArUco Marker**
- 規格：DICT_4X4_50，邊長 40mm
- 貼於機械臂末端

<img src="https://github.com/user-attachments/assets/dc9e276b-2dc3-466a-93b7-0c9006e029e4" width="500" alt="ArUco Marker 安裝">

**步驟 2：記錄標定數據**
```bash
cd eye_to_hand
python record.py
```
- 移動至 20 個不同姿態
- 按 `S` 記錄
- 按 `Q` 退出

**步驟 3：計算轉換矩陣**
```bash
python compute_transform.py
```
- 輸出 4×4 轉換矩陣
- RMSE 誤差分析
- 3D 視覺化驗證

<img src="https://github.com/user-attachments/assets/1fde736f-759d-4f46-9569-3c0a801fbab4" width="500" alt="視覺化驗證">

**步驟 4：即時測試**
- 即時轉換 ArUco 位置
- 按 `C` 輸出座標

<img src="https://github.com/user-attachments/assets/21af9c39-bf48-4fe6-9b38-3b8fe2de4ab0" width="500" alt="即時轉換">

**步驟 5：自動化標定（可選）**
```bash
python auto.py
```
⚠️ 確認工作空間安全後執行

---

## 🔧 進階設定

### TCP 校正參數調整

```python
# tcp_auto.py 中修改
SCAN_RADIUS = 15.0        # 掃描半徑 (mm)
SCAN_POINTS = 72          # 掃描點數
TOLERANCE_XY = 0.3        # XY 容差 (mm)
TOLERANCE_Z = 0.2         # Z 容差 (mm)
```

### 手眼標定參數調整

**ArUco Marker 尺寸：**
```python
marker_length = 0.04  # 單位：公尺
```

**棋盤格尺寸：**
```python
CHESSBOARD_SIZE = (9, 6)  # 內角點數量
SQUARE_SIZE = 0.025       # 方格大小 (m)
```

**相機解析度：**
```python
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
```

**機械臂 IP：**
```python
elephant_client = ElephantRobot("192.168.50.123", 5001)
```

---

## 📄 授權

MIT License - 詳見 [LICENSE](LICENSE)

---

## 👥 作者與致謝

**致謝：**
- Elephant Robotics 團隊
- Intel RealSense 團隊  
- OpenCV 社群

---

## 📧 聯絡方式

- **Email:** your.email@example.com
- **GitHub Issues:** [提交問題](https://github.com/yourusername/repo/issues)

---

## 🌟 相關資源

- [ArUco Marker 生成工具](https://chev.me/arucogen/)
- [9×6 棋盤格下載](https://github.com/opencv/opencv/blob/4.x/doc/pattern.png)

---

**最後更新：** 2026-02-02  
**版本：** v1.1.0  
**維護狀態：** 🟢 積極維護中
