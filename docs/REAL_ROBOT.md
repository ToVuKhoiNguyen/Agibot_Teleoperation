# Quy Trình Vận Hành Trên Robot Thật (Real Robot Runbook)

> [!CAUTION]
> **CẢNH BÁO NGUY HIỂM TÍNH MẠNG & THIẾT BỊ**: Mọi chuyển động trên robot thật có thể gây chấn thương nghiêm trọng hoặc phá hủy phần cứng. Mặc định hệ thống bị KHÓA CỨNG và chỉ cho phép chạy mô phỏng.

---

## 1. Năm Tầng Khóa An Toàn Phần Cứng (5 Safety Locks)
Mã nguồn robot vật lý (`teleop/robot/safety.py`) tích hợp 5 tầng bảo vệ độc lập bắt buộc phải thỏa mãn:

1. **Lock 1 - Giới hạn góc khớp (Joint Limits Clamp)**:
   - Khóa chặt biên độ góc của từng khớp tay robot, đặc biệt cổ tay: Pitch $\pm 0.45\text{ rad}$, Yaw $\pm 0.35\text{ rad}$.
2. **Lock 2 - Giới hạn vận tốc tối đa (Velocity Cap)**:
   - Khống chế vận tốc tối đa không vượt quá $0.3\text{ rad/s}$ (bước góc $\Delta \le v_{max} \cdot \Delta t$).
3. **Lock 3 - Bộ giám sát tín hiệu 300 ms (Watchdog Timer)**:
   - Nếu máy chủ không nhận được gói tin điều khiển sau $300\text{ ms}$, robot tự động ngắt ly hợp và giữ nguyên vị trí hiện tại.
4. **Lock 4 - Cổng ngắt lệch vị trí (Position Deviation Gate)**:
   - Nếu vị trí thực tế của khớp lệch so với lệnh $> 0.05\text{ rad}$ kéo dài quá $200\text{ ms}$, hệ thống lập tức kích hoạt ngắt an toàn.
5. **Lock 5 - Khóa chốt lỗi (Fault Latch & Confirm-Clear)**:
   - Khi có lỗi hoặc dừng khẩn cấp (E-STOP), hệ thống khóa chốt cứng toàn bộ lệnh điều khiển và yêu cầu người dùng xác nhận giải trừ qua lệnh/giao diện web.

---

## 2. Danh Sách Kiểm Tra Tiền Khởi Động (Pre-flight Checklist)
Trước khi khởi chạy lệnh thực tế qua `./scripts/run.sh real`, bắt buộc phải hoàn thành 4 điều kiện:
- [ ] 1. Khung treo an toàn (Gantry harness) đã được móc vào robot và tăng đơ chịu lực.
- [ ] 2. Nút dừng khẩn cấp vật lý không dây được cầm trực tiếp bởi **NGƯỜI QUAN SÁT THỨ HAI** (Second Spotter).
- [ ] 3. Khu vực làm việc của robot hoàn toàn thông thoáng, giữ khoảng cách an toàn tối thiểu 2 mét.
- [ ] 4. Pin robot đã sạc đầy, máy tính điều khiển x86 đã khởi động ổn định.

---

## 3. Quy Trình Vận Hành Chuẩn 9 Bước (S0 - S8 Runbook)

| Bước | Lệnh thực thi | Mục tiêu / Ngưỡng chấp nhận | Điều kiện hủy bỏ ngay |
| :--- | :--- | :--- | :--- |
| **S0** | `python scripts/preflight.py` | Vượt qua toàn bộ 6/6 bài kiểm tra offline | Bất kỳ bài test nào báo FAIL |
| **S1** | `python scripts/ros2_listen.py` | Tần số trạng thái tay/ngón $\ge 90\text{ Hz}$ | Không có dữ liệu sau 3 giây |
| **S2** | `python scripts/ros2_compare_home.py` | Độ lệch khớp tay $\|\Delta\| \le 0.1\text{ rad}$, ngón $< 150$ | Độ lệch góc $> 0.1\text{ rad}$ |
| **S3** | `python scripts/aima_mode.py --execute`| Chuyển controller sang Action 402 (Servo) | Lỗi RPC hoặc trạng thái $\ne 402$ |
| **S4** | `python scripts/ros2_sign_check.py --arm left` | Nhích từng khớp $+0.05\text{ rad}$ ở vận tốc $0.1\text{ rad/s}$ | Đọc ngược chiều chuyển động |
| **S5** | `./scripts/run_real.sh --arms left --no-hand` | Điều khiển tay trái qua Quest ở mức $0.3\text{ rad/s}$ | Mất tracking hoặc kích hoạt Fault |
| **S6** | `./scripts/run_real.sh --arms right --no-hand`| Điều khiển tay phải qua Quest ở mức $0.3\text{ rad/s}$ | Mất tracking hoặc kích hoạt Fault |
| **S7** | `./scripts/run_real.sh --arms both --hand-raw-max 800` | Bật cả 2 bàn tay, khống chế lực đóng tối đa 800 raw (40%) | Tín hiệu raw vượt $> 800$ |
| **S8** | `python scripts/aima_mode.py --restore` | Dừng teleop, đưa robot về thế chuẩn, nhấn E-stop | Trình tự tắt nguồn an toàn |

---

## 4. Cấu Hình Mạng & Biến Môi Trường Bắt Buộc
Robot yêu cầu dải mạng cục bộ chuyên dụng và mã Domain DDS chuẩn:
```bash
# Thiết lập Domain DDS vật lý của robot (khác hoàn toàn domain test 77)
export ROS_DOMAIN_ID=232
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```
Khi chạy `./scripts/run.sh real`, script yêu cầu nhập chính xác chuỗi xác nhận:
`I CONFIRM PREFLIGHT CHECKLIST COMPLETE`.
Biến môi trường `AGIBOT_ALLOW_REAL=1` chỉ được thiết lập nội bộ bên trong tiến trình con và tự động giải phóng khi kết thúc.
