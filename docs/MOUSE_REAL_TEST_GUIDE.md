# HƯỚNG DẪN KIỂM THỬ TELEOP CHUỘT TRÊN ROBOT THẬT (1:1 MIRROR)

Tài liệu hướng dẫn kỹ sư vận hành kiểm thử điều khiển tay robot AGIBOT A2 Ultra bằng chuột trên giao diện Web 3D (`https://localhost:8012`) kết nối robot thật qua UDP.

---

## 1. Kiểm tra tính toàn vẹn mã nguồn & SHA256

File nhận tín hiệu trên Orin (`arm_teleop_rx.py`) **KHÔNG THAY ĐỔI** (UNCHANGED).
Kiểm tra checksum trước khi chạy trên cả Laptop và Orin:
```bash
sha256sum scripts/arm_teleop_rx.py
# KẾT QUẢ BẮT BUỘC:
# dbb71f2726a3a6a307bed919e102e91f442a8076e3b9ee13021b54c0fe3b49f1  scripts/arm_teleop_rx.py
```
*(Nếu trên Orin hash khớp chuỗi trên, KHÔNG CẦN copy lại file).*

---

## 2. Thứ tự thực thi chuẩn bị (Pre-flight Checks)

### Bước 2.1: Kiểm tra tiến trình xung đột (pgrep)
Trên cả Laptop và Orin, đảm bảo không có tiến trình cũ chiếm cổng UDP 5005 / 5006:
```bash
pgrep -fl "arm_teleop_rx|main.py"
# Nếu có tiến trình dư thừa: kill -9 <PID>
sudo lsof -i :5005 -i :5006
```

### Bước 2.2: Kiểm tra xung đột ROS 2 Topic trên Orin
Đảm bảo không có node nào khác đang xuất bản lên topic điều khiển tay:
```bash
ros2 topic info /motion/control/arm_joint_command -v
# YÊU CẦU: Publisher count == 0 trước khi khởi động receiver
```

---

## 3. Khởi động hệ thống

### Bước 3.1: Khởi động Receiver trên Orin
Chạy receiver với hồ sơ tốc độ chuẩn và 5 khóa an toàn:
```bash
python3 arm_teleop_rx.py --live \
  --i-understand-hardware-risk \
  --allow-ip <IP_LAPTOP> \
  --port 5005 \
  --speed-profile normal
```
*Quan sát log:* Phải hiển thị `[arm_teleop_rx] Ready 50 Hz. LIVE=True | profile=normal (vel=0.48 rad/s)`.

### Bước 3.2: Khởi động Web Teleop trên Laptop
```bash
python3 main.py --backend real-udp \
  --speed-profile normal \
  --i-understand-hardware-risk
```
Truy cập giao diện Web: `https://localhost:8012`.

---

## 4. Thang thử nghiệm an toàn từ S0 đến S5 (Testing Ladder)

| Bậc | Nội dung kiểm thử | Thao tác thực hiện | Tiêu chí ĐẠT (PASS) | Tiêu chí DỪNG (STOP) |
|---|---|---|---|---|
| **S0** | **Tư thế khởi tạo (HOME Sync)** | Mở Web UI, kiểm tra cánh tay 3D | Sim mặc định đúng tư thế HOME. Huy hiệu hiển thị `SYNC` rồi chuyển sang `READY`. Cánh tay thật không nhúc nhích. | Robot giật khi vừa bật app; Sim lệch tư thế thật > 0.05 rad. |
| **S1** | **Đồng bộ phản hồi 5006** | Quan sát dòng telemetry trên giao diện | `Age < 200 ms`, `max\|sim-real\| < 0.02 rad`, không có cảnh báo mất gói. | Feedback age > 500 ms hoặc mất gói liên tục. |
| **S2** | **Kích hoạt Engage an toàn** | Bấm nút **[Engage Arms]** trên thanh công cụ | Trạng thái chuyển sang `ENGAGED`. Hai tay robot giữ chắc vị trí, không phát sinh xung giật. | Robot chuyển động đột ngột; xuất hiện FAULT. |
| **S3** | **Kéo chuột chậm (v <= 0.2 rad/s)** | Nhấp giữ gizmo cổ tay, kéo nhẹ nhàng | Cánh tay thật bám theo 1:1 mượt mà; độ lệch `lead < 0.035 rad`; không có rung lắc. | Khớp bị rơ, giật khục; độ lệch > 0.04 rad. |
| **S4** | **Kéo chuột nhanh (Thử nghiệm giới hạn)** | Kéo chuột bước nhảy xa (0.5 - 1.0 rad) | Gizmo vọt trước; cánh tay sim và robot thật di chuyển đồng tốc ở 0.3 rad/s; không bị văng; 0 FAULT. | Cánh tay di chuyển quá tốc độ 0.5 rad/s; xuất hiện va đập cơ khí. |
| **S5** | **Xử lý Clear Fault an toàn** | Giả lập lệch vị trí hoặc kích hoạt E-Stop rồi Clear | Bấm **[Clear Fault]**: Hệ thống chuyển về `READY`, robot đứng yên tại chỗ, KHÔNG dịch chuyển về đích cũ. Bấm **[Engage]** mới điều khiển tiếp. | Robot tự động trườn (creep) về đích cũ khi vừa bấm Clear. |

---

## 5. Giao thức E-Stop không dây (Wireless E-Stop Protocol)

1. **Vị trí nút dừng khẩn cấp:**
   - Nút E-STOP vật lý trên robot / tay điều khiển từ xa: Luôn nằm trong tay người giám sát thứ hai.
   - Nút phím tắt trên Web: Nhấn phím `Space` hoặc click nút đỏ **[EMERGENCY STOP]** ở góc phải màn hình.
2. **Khi xảy ra hiện tượng bất thường (Rung lắc, tiếng ồn lạ, lệch hướng):**
   - **BƯỚC 1:** Lập tức bấm nút E-Stop không dây hoặc nhấn `Space` trên giao diện.
   - **BƯỚC 2:** Giữ nguyên hiện trường, quan sát góc lệch hiển thị trên thanh banner đỏ.
   - **BƯỚC 3:** Không bấm Clear Fault khi chưa xác định nguyên nhân cơ học hoặc vật cản.
3. **Quy trình phục hồi sau E-Stop:**
   - Đảm bảo không gian di chuyển thông thoáng.
   - Bấm **[Clear E-Stop]** / **[Clear Fault]**.
   - Kiểm tra hệ thống tự động đồng bộ lại vị trí đo đạc thật (`READY`).
   - Bấm **[Engage Arms]** để tiếp tục quy trình kiểm thử từ bậc S2.
