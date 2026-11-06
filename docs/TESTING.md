# Hướng Dẫn Bộ Kiểm Thử Tự Động (Automated Testing Guide)

Toàn bộ hệ thống kiểm thử được thiết kế chạy trên môi trường cục bộ hoàn toàn độc lập, sử dụng ROS 2 Domain 77 và chế độ `ROS_LOCALHOST_ONLY=1` để không bao giờ phát tán gói tin ra mạng ngoài.

---

## 1. Khởi Chạy Toàn Bộ Bộ Kiểm Thử (8/8 Suites)
Chạy script tự động tổng hợp:
```bash
./scripts/run.sh test
# hoặc
bash scripts/test_all.sh
```

---

## 2. Chi Tiết Từng Bộ Kiểm Thử & Tiêu Chí Định Lượng

| Kịch bản kiểm thử | Mục tiêu kiểm định | Ngưỡng sai số định lượng |
| :--- | :--- | :--- |
| `test_mapping.py` | Kiểm tra thứ tự 14 DoF tay và 12 DoF ngón, URDF sign alignment | Cả 12 khớp mở $=0$; đóng $[1815..1877]$ raw; sai số $=0$ |
| `test_ros2_backend.py` | Kiểm định độ tinh khiết không-ROS khi import, 4 cổng khóa bảo vệ | Tần số $45-55\text{ Hz}$, độ chính xác $10^{-6}$, watchdog ngắt an toàn |
| `test_robot_safety.py` | Kiểm tra 5 tầng khóa an toàn: clamp giới hạn, vận tốc, watchdog | Cổ tay pitch $\pm 0.45$, yaw $\pm 0.35$; bước góc $\le v_{max}\cdot dt$ |
| `test_real_suite.py` | Kiểm tra cô lập chuyển động đơn khớp, giới hạn lực đóng bàn tay | Góc dịch chuyển tối đa $\le 0.05\text{ rad}$; lực tay $\le 800$ raw |
| `test_web_sync.py` | Đồng bộ vị trí thực tế MuJoCo và Streamed Link Poses | Sai số BBox 72 visual geoms $< 1.0\text{ mm}$; sai số link $< 1.0\text{ mm}$ |
| `test_panel_api.py` | Kiểm tra toàn diện REST API: Jog, Preset, E-STOP, Gizmo Yaw | Sai số jog $< 2.0\%$; sai số góc $< 3.0^\circ$; dừng tức thời $0\text{ mm}$ |
| `test_xr_controllers.py` | Kiểm tra 10 tính năng tay cầm Quest: 4-state clutch, panic open | Sai số dịch chuyển chuyển trạng thái $< 2.0\text{ mm}$, góc $< 0.5^\circ$ |
| `test_handtracking.py` | Kiểm định theo dõi bàn tay 1:1, bộ lọc OneEuro, head yaw/pitch | Độ trễ $\le 60\text{ ms}$; nhiễu RMS $\le 1.0\text{ mm}$; giật phục hồi $< 2\text{ mm}$ |

---

## 3. Kiểm Thử Giao Diện Đồ Họa Headless (Playwright)
Để kiểm tra giao diện Web 3D và các chỉ số đo lường hiệu năng FPS/Draw Calls mà không cần mở trình duyệt thật:
```bash
python scripts/test_handtracking.py
```
Kịch bản tự động kích hoạt Playwright headless kết nối tới `https://localhost:8012/` (bỏ qua cảnh báo chứng chỉ SSL tự ký) và ghi nhận ảnh chụp màn hình kiểm định vào `/tmp/shots/`.

---

## 4. Tạo Báo Cáo Kết Quả Tự Động
Để xuất báo cáo markdown tổng hợp kết quả chạy kiểm thử, độ trễ và mã băm commit Git:
```bash
./scripts/run.sh report
```
Báo cáo được lưu tự động tại thư mục `reports/report_<timestamp>.md`.
