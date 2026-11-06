# Hướng Dẫn Chi Tiết Các Chế Độ Vận Hành (Operating Modes)

Tài liệu này hướng dẫn chi tiết quy trình khởi chạy, tương tác và tiêu chí kiểm định định lượng cho từng chế độ vận hành trong hệ thống AGIBOT A2 Ultra Teleoperation.

---

## 1. Chế Độ Mô Phỏng 3D Chuột (`sim`)
- **Lệnh chạy**: `./scripts/run.sh sim`
- **Giao diện**: `https://localhost:8012/`
- **Mục đích**: Vận hành mô phỏng vật lý MuJoCo với digital twin 3D, thao tác trực tiếp qua con trỏ chuột máy tính.
- **Cách điều khiển**:
  - Kéo các tay cầm gizmo PivotControls tại vị trí cổ tay Trái/Phải để điều khiển 6-DoF (Dịch chuyển theo trục X/Y/Z, mặt phẳng XY/YZ/XZ, xoay quanh các cung góc, hoặc kéo khối cầu trung tâm).
  - Sử dụng thẻ **Preset Motions & Demos** để kích hoạt các quỹ đạo mẫu: `Home`, `Arms Fwd`, `Wave`, v.v.
  - Sử dụng nút **EMERGENCY STOP** ở đầu thẻ điều khiển để dừng khẩn cấp toàn bộ hệ thống ngay lập tức.
- **Tiêu chí định lượng**:
  - Sai số bám vị trí IK so với MuJoCo: $< 1.0\text{ mm}$.
  - Sai số BBox giữa 72 visual geoms và vật lý: $< 1.0\text{ mm}$.

---

## 2. Chế Độ Tay Cầm Meta Quest 3S (`controllers`)
- **Lệnh chạy**: `./scripts/run.sh controllers`
- **Giao diện WebXR**: `https://localhost:8012/?input=controllers`
- **Kết nối Quest qua USB**:
  ```bash
  adb reverse tcp:8012 tcp:8012
  ```
- **Bảng Ánh Xạ Phím Điều Khiển (Đã Đảo Trigger ↔ Grip)**:

| Phím vật lý | Trạng thái ly hợp | Hành động / Chức năng |
| :--- | :--- | :--- |
| **Index Trigger (Giữ)** | `TRANSLATE` | Dịch chuyển vị trí 3D $(x, y, z)$; góc xoay được KHÓA cứng |
| **Side Grip (Giữ)** | `ROTATE` | Xoay hướng 3D $(R)$; vị trí 3D được KHÓA cứng |
| **Cả Trigger + Grip (Giữ)** | `6D` | Điều khiển toàn phần 6-DoF cả vị trí và góc xoay đồng thời |
| **Thả cả hai phím** | `LOCKED` | Khóa mục tiêu tại chỗ, tay robot giữ nguyên vị trí |
| **Thumbstick Y (Đẩy/Kéo)** | Co/Duỗi ngón | Điều khiển vận tốc đóng/mở bàn tay; tự giữ mức đóng khi thả cần |
| **Thumbstick Click (Nhấn)** | `PANIC OPEN` | Mở khẩn cấp bàn tay về 0% ngay lập tức |
| **Thumbstick X (Gạt ngang)**| Đổi kiểu nắm | Chuyển đổi giữa các preset: CYLINDER, PINCH, SPHERE, HOOK |
| **Nút A / X (Giữ 1.5 s)** | `HOME` | Tự động đưa cánh tay về tư thế chuẩn bị ban đầu |
| **Nút B / Y (Nhấn nhả)** | `PRECISION` | Bật/tắt chế độ điều khiển tinh chỉnh (hệ số tỉ lệ $\times 0.30$) |
| **Nút B + Y (Giữ 0.5 s)** | `E-STOP` | Tổ hợp phím kích hoạt dừng khẩn cấp phần mềm |

- **Tiêu chí định lượng**:
  - Độ dịch chuyển tức thời khi chuyển trạng thái ly hợp: $< 2.0\text{ mm}$, sai số góc $< 0.5^\circ$.
  - Watchdog tự ngắt ly hợp về `LOCKED` khi mất tín hiệu tracking $> 350\text{ ms}$.

---

## 3. Chế Độ Bàn Tay 1:1 Tuyệt Đối (`hands`)
- **Lệnh chạy**: `./scripts/run.sh hands`
- **Giao diện WebXR**: `https://localhost:8012/?input=hands`
- **Tính năng nổi bật**:
  - Thu nhận trực tiếp 25 khớp xương bàn tay `XRHand` từ Meta Quest qua WebXR.
  - Bộ lọc OneEuro thời gian thực: Độ trễ $\le 60\text{ ms}$, sai số nhiễu RMS $\le 1.0\text{ mm}$.
  - Giải quyết hoàn toàn lỗi co tay vô tình trên Quest (cả tay mở và nắm đấm đều dịch chuyển cánh tay đúng tỉ lệ $10\text{ cm} \pm 5\text{ mm}$).
  - Ánh xạ góc Yaw/Pitch của đầu người vận hành vào 2 khớp cổ robot (Actuators 26/27).
- **Cơ chế khóa an toàn máy thật**: Chế độ điều khiển bàn tay thuần túy bị TỪ CHỐI kích hoạt trên robot vật lý nhằm đảm bảo an toàn tuyệt đối.

---

## 4. Các Chế Độ Ảo & Kiểm Tra Dòng Dữ Liệu
### 4.1. Chế độ Dry-Run (`dryrun`)
- **Lệnh chạy**: `./scripts/run.sh dryrun`
- Chạy toàn bộ chu trình teleop và xuất bảng đo lường CLI định kỳ 50 Hz mà không can thiệp vào mô phỏng vật lý hay socket mạng thật.

### 4.2. Chế độ ROS 2 Dry (`ros2dry`)
- **Lệnh chạy**: `./scripts/run.sh ros2dry`
- Bật node ROS 2 xuất bản các chủ đề dữ liệu ảo, giúp kiểm tra luồng dữ liệu DDS mà không kích hoạt động cơ robot.

### 4.3. Chế độ Phát Lại Quỹ Đạo (`replay`)
- **Lệnh chạy**: `./scripts/run.sh replay <đường_dẫn_file>`
- Nạp quỹ đạo khớp đã ghi lại trước đó để kiểm tra chuyển động bám mẫu trên mô phỏng.
