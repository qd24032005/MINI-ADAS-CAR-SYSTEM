# Mini ADAS – Hệ thống phát hiện và cảnh báo tiền va chạm cho xe mô hình

## 1. Giới thiệu dự án

Dự án này mô phỏng một hệ thống **Mini ADAS (Advanced Driver Assistance System)** cho xe mô hình sử dụng vi điều khiển STM32.  
Hệ thống có khả năng đo khoảng cách vật cản phía trước, phân loại mức độ nguy hiểm, cảnh báo bằng LED/buzzer, hiển thị trạng thái lên LCD và điều khiển motor để hỗ trợ tránh va chạm.

Mục tiêu chính của dự án là xây dựng một hệ thống nhúng có khả năng:

- Đo khoảng cách vật cản phía trước bằng cảm biến siêu âm.
- Cảnh báo theo 3 mức: SAFE, WARNING, DANGER.
- Hiển thị chế độ hoạt động và khoảng cách trên LCD.
- Điều khiển xe ở hai chế độ: Auto Mode và Manual Mode.
- Tự động phanh/dừng khi phát hiện vật cản trong vùng nguy hiểm.
- Nhận lệnh điều khiển từ Bluetooth/UART.

---

## 2. Chức năng chính

### 2.1. Đo khoảng cách vật cản

Hệ thống sử dụng cảm biến siêu âm HC-SR04 để đo khoảng cách phía trước xe.  
STM32 tạo xung TRIG và dùng Timer Input Capture để đo độ rộng xung ECHO, từ đó tính ra khoảng cách theo đơn vị cm.

### 2.2. Phân loại trạng thái an toàn

Khoảng cách đo được được chia thành 3 vùng:

| Trạng thái | Điều kiện khoảng cách | Phản ứng |
|---|---|---|
| SAFE | >= 100 cm | Bật LED an toàn |
| WARNING | 50 cm – 100 cm | Bật LED cảnh báo |
| DANGER | < 50 cm | Bật LED nguy hiểm và buzzer |

### 2.3. Auto Mode

Ở chế độ Auto Mode, xe tự động chạy tiến với tốc độ cố định.  
Khi khoảng cách tới vật cản nhỏ hơn khoảng cách an toàn, xe sẽ tự động phanh/lùi nhẹ rồi dừng lại.

### 2.4. Manual Mode

Ở chế độ Manual Mode, người dùng điều khiển xe thông qua Bluetooth/UART.  
Tuy nhiên, hệ thống vẫn luôn kiểm tra khoảng cách phía trước. Nếu vật cản quá gần, lệnh đi tiến sẽ bị chặn và xe sẽ tự động phanh để tránh va chạm.

### 2.5. Hiển thị LCD

LCD 16x2 hiển thị:

- Chế độ hiện tại: BOOT, AUTO hoặc MANUAL.
- Khoảng cách đo được.
- Trạng thái an toàn: SAFE, WARN hoặc DANGER.

Ví dụ:

```text
MODE: AUTO
DIST:75cm WARN
