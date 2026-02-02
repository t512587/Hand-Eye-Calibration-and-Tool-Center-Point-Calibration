# 🤖 Elephant Robotics 手眼標定

[![Python Version](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Robot](https://img.shields.io/badge/Robot-Elephant_Robotics-orange.svg)](https://www.elephantrobotics.com/)
[![Camera](https://img.shields.io/badge/Camera-Intel_RealSense-00C9FF.svg)](https://www.intelrealsense.com/)

本專案提供一套為 **大象機器人 (Elephant Robotics)** 與 **Intel RealSense D400 系列相機** 設計的完整手眼標定解決方案，支援 **Eye-in-Hand（眼在手）** 與 **Eye-to-Hand（眼不在手）** 兩種配置模式。

---

## 📖 目錄

- [系統架構](#-系統架構)
- [硬體配置與環境](#-硬體配置與環境)
- [安裝指南](#-安裝指南)
- [專案結構](#-專案結構)
- [操作流程](#-操作流程)
  - [Eye-in-Hand 模式](#1-eye-in-hand-眼在手模式)
  - [Eye-to-Hand 模式](#2-eye-to-hand-眼不在手模式)
- [進階設定](#-進階設定)
- [授權](#-授權)
- [作者與致謝](#-作者與致謝)
- [聯絡方式](#-聯絡方式)
- [延伸閱讀](#-延伸閱讀)

---

## 🏗 系統架構

本專案支援兩種手眼標定配置：

### Eye-in-Hand（眼在手）
相機固定於機械臂末端，隨手臂移動。
```
機械臂末端 → 相機 → 觀測目標（棋盤格標定板 固定於工作台）
```


### Eye-to-Hand（眼不在手）
相機固定於外部位置（如支架），觀測機械臂末端的標記物。
```
外部相機 → 觀測 → 機械臂末端（ArUco Marker 貼於末端）
```


---

## 🛠 硬體配置與環境

### 硬體需求

| 組件 | 規格 | 備註 |
|------|------|------|
| **機械臂** | Elephant Robotics 系列 | 如 myCobotpro630 等 |
| **相機** | Intel RealSense D415/D435/D455 | 支援深度資訊 |
| **標記物** | ArUco Marker (DICT_4X4_50) 或 棋盤格 (9×6) | Eye-to-Hand 用 ArUco，Eye-in-Hand 用棋盤格 |
| **連線方式** | TCP/IP | 預設 IP: `192.168.50.123`, Port: `5001` |

### 軟體環境

- **Python**: 3.8 以上

---

## 📦 安裝指南

### 安裝依賴套件
```bash
pip install numpy opencv-contrib-python pyrealsense2 pymycobot scipy matplotlib
```

### 驗證安裝
```bash
python -c "import cv2, pyrealsense2, numpy; print('安裝成功！')"
```

---

## 📁 專案結構
```
Hand-Eye-Calibration/
├── eye_in_hand/                    # 眼在手模式（相機在末端）
│   ├── record.py                   # 手動記錄標定數據（使用9x6棋盤格）
│   ├── compute_transform.py        # 計算轉換矩陣與即時測試
│   ├── auto.py                     # 自動化標定流程
│   └── handeye_records/            # 記錄數據儲存目錄
│       └── handeye_chessboard_*.json
│
├── eye_to_hand/                    # 眼不在手模式（相機固定外部）
│   ├── record.py                   # 手動記錄標定數據（使用 ArUco）
│   ├── compute_transform.py        # 計算轉換矩陣與即時測試
│   ├── auto.py                     # 自動化標定流程
│   └── aruco_records/              # 記錄數據儲存目錄
│       └── complete_record_*.json
│
├── 介紹.txt                        # 專案說明文件
└── README.md                       # 本文件
```

---

## 🚀 操作流程

## 1. Eye-in-Hand 眼在手模式

> **配置說明：** 相機固定於機械臂末端，棋盤格放置工作台固定位置

### 步驟 1: 準備標定板

準備 **棋盤格標定板（9×6 內角點，方格大小 25mm）**

### 步驟 2: 手動記錄標定數據
```bash
cd eye_in_hand
python record.py
```

**操作說明：**
- 將棋盤格放置於工作台不同位置和角度
- 或手持棋盤格在相機視野內移動
- 確保相機能清楚檢測到棋盤格角點
- 按 `S` 鍵記錄棋盤格姿態 + 機械臂初始姿態
- 按 `M` 鍵記錄該點移動後的姿態（可選）
- 按 `V` 鍵查看已記錄資料
- 按 `R` 鍵重置資料
- 按 `Q` 鍵離開並儲存

**輸出檔案：** `handeye_records/handeye_chessboard_YYYYMMDD_HHMMSS.json`

### 步驟 3: 計算轉換矩陣

編輯 `compute_transform.py`，修改 JSON 檔案路徑：
```python
CALIBRATION_DATA_JSON_PATH = r"handeye_records/handeye_chessboard_20260119_132556.json"
```

執行計算：
```bash
python compute_transform.py
```

**計算過程：**
1. 載入標定數據
2. 使用 **TSAI 方法** 計算手眼標定
3. 驗證標定精度
4. 輸出相機到夾爪的變換矩陣

```

### 步驟 4: 即時座標轉換測試

執行後選擇啟動實時系統：
```bash
python compute_transform.py
# 輸入 JSON 路徑（或使用預設）
# 輸入機器人 IP（或使用預設 192.168.50.123）
# 輸入 'y' 啟動實時系統
```

**功能：**
- 即時檢測棋盤格位置
- 轉換為機械臂基座座標系
- 按 `C` 鍵輸出當前座標
- 按 `Q` 鍵退出

---

## 2. Eye-to-Hand 眼不在手模式

> **配置說明：** 相機固定於外部支架，ArUco Marker 貼於手臂末端

### 步驟 1: 準備標記物

將 **ArUco Marker（ID: DICT_4X4_50, 邊長 40mm）** 貼於機械臂末端，如下圖所示：

<img src="https://github.com/user-attachments/assets/dc9e276b-2dc3-466a-93b7-0c9006e029e4" width="600" alt="ArUco Marker 安裝示意圖">


### 步驟 2: 手動記錄標定數據
```bash
cd eye_to_hand
python record.py
```

**操作說明：**
- 手動移動機械臂至不同姿態（建議 15 個點位）
- 確保每個姿態下相機都能清楚檢測到 ArUco Marker
- 按 `S` 鍵記錄當前檢測到的 ArUco 標記
- 按 `R` 鍵重置記錄
- 按 `V` 鍵查看已記錄的點位
- 按 `Q` 鍵退出程式

**輸出檔案：** `aruco_records/complete_record_YYYYMMDD_HHMMSS.json`

### 步驟 3: 計算轉換矩陣

編輯 `compute_transform.py`，修改 JSON 檔案路徑：
```python
transformer = CameraToRobotTransform(r"aruco_records/complete_record_20260119_143025.json")
```

執行計算：
```bash
python compute_transform.py
```

**輸出結果：**
1. **4×4 轉換矩陣** - 相機到機械臂基座的座標轉換
2. **RMSE 誤差分析** - 標定精度評估
3. **3D 視覺化圖** - 驗證轉換矩陣準確性

<img src="https://github.com/user-attachments/assets/1fde736f-759d-4f46-9569-3c0a801fbab4" width="600" alt="空間視覺化驗證圖">


### 步驟 4: 即時座標轉換測試

執行即時轉換系統：
```bash
python compute_transform.py
# 選擇啟動實時系統
```

**功能展示：**
- 即時顯示 ArUco Marker 在相機座標系的位置
- 即時轉換為機械臂基座座標系
- 按 `C` 鍵輸出當前座標
- 按 `Q` 鍵退出

<img src="https://github.com/user-attachments/assets/21af9c39-bf48-4fe6-9b38-3b8fe2de4ab0" width="600" alt="即時轉換介面">

### 步驟 5: 自動化標定（可選）
```bash
python auto.py
```

**選項 1 - 自動化標定：**
- 機械臂自動移動至 15 個預設點位
- 相機自動識別 ArUco 標記
- 生成標定數據檔案 `automated_calibration_data.json`

**選項 2 - 即時轉換測試：**
- 基於已完成的標定結果
- 即時顯示標記在機械臂基座座標系下的 XYZ 數值

> ⚠️ **安全警告：** 自動標定前請確認機械臂工作半徑內無障礙物！

---

## 🔧 進階設定

### 自訂 ArUco Marker 尺寸

在 `record.py` 中修改：
```python
# ArUco 設定
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
marker_length = 0.04  # 單位：公尺（修改此處）
```

### 自訂棋盤格尺寸

在 `record.py` 和 `compute_transform.py` 中修改：
```python
# 棋盤格設定
CHESSBOARD_SIZE = (9, 6)  # 內角點數量 (列, 行)
SQUARE_SIZE = 0.025  # 每個方格的實際大小，單位: 公尺 (25mm)
```

### 調整相機解析度
```python
# 在 record.py 中修改
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# 可改為: 1280x720, 320x240 等
```

### 更改機械臂 IP
```python
# 在所有 .py 檔案中修改
elephant_client = ElephantRobot("192.168.50.123", 5001)
# 改為您的機械臂 IP
```

### 調整標定點位數量
```python
# 在 record.py 中修改
target_points = 15  # 修改為您想要的數量（建議 15-30）
```

---

## 📄 授權

本專案採用 MIT 授權條款 - 詳見 [LICENSE](LICENSE) 檔案
```
MIT License

Copyright (c) 2026 Your Name

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 👥 作者與致謝

**作者：** Your Name  
**機構：** Your Organization  
**專案起始：** 2026 年 1 月

**致謝：**
- **Elephant Robotics 團隊** - 提供優秀的機械臂硬體和技術支援
- **Intel RealSense 團隊** - 提供強大的深度相機 SDK
- **OpenCV 社群** - 提供完善的電腦視覺算法庫
- **所有貢獻者** - 感謝所有提供意見和程式碼的開發者

**參考文獻：**

---

## 📧 聯絡方式

- **Email:** your.email@example.com
- **GitHub Issues:** [提交問題](https://github.com/yourusername/elephant-handeye-calibration/issues)
- **討論區:** [GitHub Discussions](https://github.com/yourusername/elephant-handeye-calibration/discussions)
- **官方網站:** https://yourwebsite.com
- **技術支援:** support@yourwebsite.com

---

## 🌟 如果這個專案對您有幫助，請給我們一個 Star！

[![Star History Chart](https://api.star-history.com/svg?repos=yourusername/elephant-handeye-calibration&type=Date)](https://star-history.com/#yourusername/elephant-handeye-calibration&Date)

---

### 相關資源
- [ArUco Marker 生成工具](https://chev.me/arucogen/)
- [9x6棋盤格下載](https://github.com/opencv/opencv/blob/4.x/doc/pattern.png)
---
**最後更新：** 2026-01-19  
**版本：** v1.0.0  
**維護狀態：** 🟢 積極維護中

---

<div align="center">



</div>
