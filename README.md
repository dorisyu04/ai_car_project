# 🚗 AI Car Project (自走車系統)

## 📌 專案簡介
本專案為一套嵌入式 AI 自走車系統，整合 Raspberry Pi、OpenCV、Flask 與 YOLO，
實現「影像辨識 + 自動循線 + 網頁遠端控制」的完整系統。

透過瀏覽器即可即時操控車輛，並同步觀看攝影機畫面與 AI 判斷結果。

---

## 🚀 系統功能

### 🔹 車道線偵測 (Lane Detection)
- 使用 OpenCV 進行影像處理
- 灰階轉換 + 高斯模糊 + 邊緣偵測
- ROI 區域限制降低雜訊
- 計算車道中心位置

### 🔹 PID 自動控制
- 計算車道偏差 (error)
- 使用 PID 控制器修正方向
- 控制左右輪速實現穩定循線

### 🔹 YOLO 物件辨識
- 即時辨識交通號誌（紅綠燈等）
- 顯示 Bounding Box
- 可延伸紅燈停車功能

### 🔹 Web 遙控系統
- Flask 建立 Web Server
- 即時影像串流 (MJPEG)
- 支援前進 / 後退 / 左右轉
- AJAX 非同步控制（不卡畫面）

---

## 🧠 系統架構

```
Camera (PiCamera2)
↓
Image Processing (OpenCV)
↓
Lane Detection / YOLO
↓
PID Controller
↓
Motor Control (PCA9685)
↓
Flask Web Server
↓
Browser UI (Remote Control)
```


---

## 📂 專案結構


ai_car_project/
├── app.py # Flask 主程式（影像串流 + 控制）
├── motor_control.py # 馬達控制模組
├── pid_controller.py # PID 控制器
├── templates/
│ └── index.html # 前端 UI
└── static/
├── css/style.css # 網頁樣式
└── js/main.js # 控制邏輯
---

## ⚙️ 執行方式

### 1️⃣ 安裝套件
```bash
pip install Flask opencv-python ultralytics

2️⃣ 啟動系統
python app.py

3️⃣ 開啟瀏覽器
http://<RaspberryPi_IP>:5000
