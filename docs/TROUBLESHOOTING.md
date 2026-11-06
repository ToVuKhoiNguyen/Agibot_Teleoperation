# Khắc Phục Sự Cố, Điểm Chưa Xác Định & Vệ Sinh Phát Hành

---

## 1. Ma Trận Khắc Phục Sự Cố Thường Gặp (Troubleshooting Matrix)

| Hiện tượng | Nguyên nhân gốc | Hướng xử lý |
| :--- | :--- | :--- |
| **Không nhận được chủ đề ROS 2** | Robot chưa ở chế độ Servo hoặc sai dải IP | Ping `192.168.100.100`; chạy lệnh chuyển chế độ `scripts/aima_mode.py` |
| **Sai lệch Domain DDS** | Biến `ROS_DOMAIN_ID` không khớp máy thật | Thiết lập đúng `export ROS_DOMAIN_ID=232` (hoặc ID do hãng cung cấp) |
| **FastDDS không tìm thấy nhau** | Gói tin multicast bị chặn giữa Jazzy và Humble | Thiết lập biến môi trường `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` |
| **Mất tín hiệu điều khiển (Watchdog)** | Kính Quest mất tầm nhìn hoặc tay cầm bị che | Ly hợp tự động khóa sau $350\text{ ms}$; đưa tay về góc nhìn để bắt lại |
| **Kích hoạt chốt lỗi FAULT** | Dữ liệu trễ $> 200\text{ ms}$ hoặc lệch góc $> 0.05\text{ rad}$ | Giữ nguyên robot; bấm "Clear Fault" trên giao diện Web để mở lại |
| **Không có nút ENTER VR trên Quest** | Kết nối qua HTTP thường hoặc trình duyệt lạ | Truy cập bằng HTTPS: `https://localhost:8012/` trên Meta Quest Browser |
| **Lỗi cổng 8012 đã bị chiếm dụng** | Một tiến trình python cũ vẫn đang chạy ngầm | Chạy `fuser -k 8012/tcp` hoặc kiểm tra PID qua `ss -tulpn \| grep 8012` |
| **Lỗi import rclpy trong venv** | Môi trường ảo Python bị cô lập khỏi ROS 2 | Khởi tạo lại venv với cờ `--system-site-packages` |

---

## 2. Danh Sách Các Điểm Chưa Xác Định Từ Nhà Sản Xuất (Known Unknowns)
1. **Thời gian ngắt của Hardware Watchdog trên robot**: Tài liệu kỹ thuật của hãng không công bố rõ thời gian ngắt; mã điều khiển teleop truyền ở tần số 50 Hz và tự ngắt nếu gói tin trễ quá $200\text{ ms}$.
2. **Cơ chế ngắt mạch của nút E-Stop phần cứng**: Cần trang bị nút dừng khẩn cấp không dây riêng; giả định động cơ mất nguồn và cánh tay thả lỏng $\to$ bắt buộc phải dùng khung gantry đỡ trọng lượng.
3. **Chiều dương của khớp xoay ngón cái (`thumb_0`)**: Giao thức AimDK không chỉ rõ; ngón cái được giữ ở vị trí trung tính ($0\text{ raw}$) trừ khi truyền cờ `--enable-thumb-swing`.
4. **Động học tay đẩy thanh giằng cổ tay**: Mô hình động học nghịch (IK) giả định tương đương với cơ cấu nối tiếp chuẩn.
5. **Giới hạn tải trọng**: Tải trọng danh định 1 kg một tay / 3 kg lực kẹp; phạm vi với tối đa chưa được nhà sản xuất chuẩn hóa.
6. **Bản quyền file lưới 3D**: Toàn bộ file lưới STL/OBJ trong thư mục `assets/` được trích xuất phục vụ mục đích nghiên cứu học thuật nội bộ, chưa được xác nhận giấy phép thương mại từ hãng.
7. **Trạng thái trên phần cứng vật lý**: Toàn bộ hệ thống chưa từng được vận hành trên máy thật; mọi thử nghiệm mới dừng lại ở mô phỏng và kiểm tra mạng cục bộ.

---

## 3. Quy Tắc Vệ Sinh Phát Hành & Quy Trình Git (Hygiene per AGENTS.md)
- **Tuyệt đối không push tùy tiện**: Chỉ thực hiện `git push` khi người dùng yêu cầu rõ ràng.
- **Không bao giờ dùng `--force` hoặc viết lại lịch sử remote**.
- **Kiểm soát dung lượng và bí mật**:
  - Không commit bất kỳ file nào có dung lượng $> 5\text{ MB}$.
  - Không commit file khóa, chứng chỉ (`*.key`, `*.pem`), biến môi trường (`.env`), thư mục tạm `venv/`, `_trash/`, `logs/`.
- **Kiểm tra độ dài tệp tin nguồn**: Toàn bộ các file mã nguồn và tài liệu phải tuân thủ giới hạn $\le 250$ dòng (`python scripts/check_file_sizes.py`).
- **Quy chuẩn thông điệp Commit**: Sử dụng cú pháp Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, `refactor:`).
