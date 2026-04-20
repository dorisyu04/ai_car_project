# AI Car Project (自走車系統)

## 專案簡介
本專案為嵌入式 AI 自走車系統，整合 Raspberry Pi、OpenCV、Flask 與 YOLO 技術，
實現即時影像辨識、自動循線與網頁遠端控制功能。

---

## 系統功能
- 車道線偵測（OpenCV）
- PID 自動控制
- YOLO 物件辨識
- Flask 網頁遙控

---

## 系統架構
Camera → Image Processing → Lane Detection → PID → Motor Control → Web

---

## 專案結構
ai_car_project/
├── app.py
├── motor_control.py
├── pid_controller.py
├── templates/
│   └── index.html
└── static/
    ├── css/style.css
    └── js/main.js

---

## 執行方式
pip install Flask opencv-python ultralytics

python app.py

開啟瀏覽器：
http://<RaspberryPi_IP>:5000

---

## 作者
dorisyu04
