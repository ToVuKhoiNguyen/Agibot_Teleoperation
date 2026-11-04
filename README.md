# AGIBOT A2 Ultra - Hệ thống Điều khiển Từ xa XR & Mô phỏng Song sinh Kỹ thuật số (Digital Twin)

Hệ thống điều khiển từ xa (Teleoperation) độ trễ thấp và song sinh số thời gian thực cho robot hình người hai tay AGIBOT A2 Ultra kết hợp bàn tay khéo léo OmniHand (5 ngón).

---

## 1. Giới thiệu

- **Mục tiêu**: Cung cấp giải pháp teleoperation trực quan qua kính thực tế ảo Meta Quest 3S (WebXR) hoặc giao diện web 3D trên PC, giải thuật động học ngược (Damped Least-Squares IK), retargeting cánh tay và bàn tay khéo léo, đồng bộ song sinh số 3D.
- **An toàn cốt lõi**: **Chỉ chạy mô phỏng vật lý (MuJoCo) theo mặc định**. Phần cứng robot thật hoàn toàn bị khóa ở mức logic và cấu hình.

---

## 2. Yêu cầu hệ thống và cài đặt

- **Hệ điều hành**: Linux (Ubuntu 22.04 / 24.04 LTS).
- **Python**: Khuyến nghị Python 3.10 - 3.12 trong môi trường ảo `venv`.
- **Phụ thuộc**: `pip install -r requirements.txt` (MuJoCo, AioHTTP, PyYAML, NumPy, SciPy, Trimesh...).
- **ADB**: Cài đặt Android Debug Bridge (`sudo apt install adb`) để kết nối có dây USB với kính Quest 3S.

```bash
cd ~/a2_ws
source venv/bin/activate
pip install -r requirements.txt
```

---

## 3. Chạy nhanh (Quick Start)

Chỉ 3 dòng lệnh để khởi động toàn bộ hệ thống và mở giao diện điều khiển:

```bash
cd ~/a2_ws
source venv/bin/activate
python main.py
```
Mở trình duyệt trên máy tính và truy cập: **`https://localhost:8012/`**

---

## 4. Bảng các cờ dòng lệnh của main.py

Xác minh trực tiếp từ `python main.py --help`:

| Cờ lệnh | Kiểu | Mặc định | Mô tả |
| :--- | :---: | :---: | :--- |
| `-h`, `--help` | Cờ | - | Hiển thị hướng dẫn và danh sách các tùy chọn dòng lệnh |
| `--port PORT` | `int` | `8012` | Cổng máy chủ HTTPS & WebSocket server |
| `--viewer` | Cờ | `Bật nếu có display` | Kích hoạt cửa sổ native GUI viewer của MuJoCo (tự động bật khi có `$DISPLAY` hoặc `$WAYLAND_DISPLAY`) |
| `--no-viewer` | Cờ | `False` | Tắt cửa sổ native GUI viewer của MuJoCo |
| `--headless` | Cờ | `False` | Chạy ở chế độ không màn hình (ép tắt native viewer của MuJoCo) |
| `--controller` | Cờ | `False` | Chế độ teleop bằng tay cầm Quest (clutch mode bằng nút Grip, bóp ngón bằng Trigger) |
| `--xr-view` | Cờ | `False` | Bật mô hình song sinh số 3D bên trong không gian kính thực tế ảo Quest |
| `--backend` | `str` | `mujoco` | Backend vật lý: `mujoco` hoặc `replay` |
| `--speed-scale` | `float` | `1.0` | Hệ số tỷ lệ tốc độ mô phỏng thời gian thực |

---

## 5. Điều khiển bằng chuột trên Web (Web Control Panel)

Truy cập `https://localhost:8012/` (tự động chuyển hướng đến `/panel`). Giao diện chia làm hai khu vực: **Khung nhìn 3D Digital Twin** (bên trái) và **Bảng điều khiển chuột** (bên phải):

- **Arm Jog (Tay Trái / Tay Phải)**: Các nút `+X, -X, +Y, -Y, +Z, -Z` (hệ toạ độ robot: X tiến trước, Y sang trái, Z lên trên) và điều chỉnh góc `Roll / Pitch / Yaw`. Nhấn giữ chuột để lặp lại bước jog.
- **Hand Control**: Các nút `Open` (mở tay), `Close` (nắm tay), `Pinch` (chụm ngón) và thanh trượt độ khép ngón từ 0.0 đến 1.0.
- **Preset Poses**:
  - `Home`: Đưa robot về tư thế đứng chuẩn bị hai tay cong trước ngực.
  - `Arms Forward`, `Arms Up`, `Arms Out`: Các tư thế mẫu nhanh.
  - `Wave Hand`: Vẫy tay phải theo quỹ đạo hình sin chu kỳ 3 giây.
  - `Arms Circle`: Hai cánh tay chuyển động theo quỹ đạo tròn đồng bộ.
  - `Walk Demo`: Mô phỏng bước chân đi bộ tại chỗ của hai chân robot.
- **3D Transform Gizmo**: Nhấp chọn quả cầu mục tiêu cổ tay (Target Marker L/R) trong khung nhìn 3D và kéo các trục gizmo để điều khiển vị trí trực tiếp bằng chuột qua giải thuật DLS IK.
- **An toàn khẩn cấp**:
  - Nút **E-STOP** đỏ lớn: Dừng khẩn cấp toàn bộ chuyển động, ngắt lệnh điều khiển.
  - Nút **Clear E-STOP**: Hủy trạng thái dừng khẩn cấp để tiếp tục điều khiển.
  - Nút **Hold Arms**: Khóa cứng vị trí hiện tại của cánh tay.
  - Nút **Reset**: Đưa toàn bộ robot về keyframe home ban đầu.

---

## 6. Kết nối Quest 3S qua USB & WebXR

1. **Kết nối USB-C**: Cắm kính Meta Quest 3S vào máy tính bằng cáp USB 3.0 và bật tính năng USB Debugging.
2. **Chuyển tiếp cổng qua ADB**:
   ```bash
   adb reverse tcp:8012 tcp:8012
   ```
3. **Mở trình duyệt Quest Browser**:
   ```bash
   adb shell am start -a android.intent.action.VIEW -d "https://localhost:8012/?ws=wss://localhost:8012"
   ```
4. **Bắt đầu Teleop**:
   - Chấp nhận cảnh báo chứng chỉ SSL tự ký (chọn *Advanced -> Proceed to localhost*).
   - Nhấn nút **Enter VR** trên trình duyệt kính để bắt đầu phiên WebXR.
   - **Chế độ Hand Tracking**: Di chuyển hai bàn tay tự nhiên trước kính; cổ tay và các khớp ngón tay OmniHand sẽ bám theo chuyển động thực.
   - **Chế độ Controller (`--controller`)**: 
     - Giữ nút **Grip** (bên hông) để kích hoạt clutch điều khiển vị trí và góc quay của cổ tay robot; nhả nút Grip để thả tự do tay người điều khiển mà robot vẫn giữ nguyên tư thế.
     - Bóp phím **Trigger** để đóng mở bàn tay khéo léo.

---

## 7. Cấu trúc thư mục

```
agibot_xr_teleoperate/
├── assets/                  # Mô hình robot AGIBOT A2 Ultra (MJCF, URDF, meshes STL và web_meshes)
├── certs/                   # Chứng chỉ SSL HTTPS tự sinh (cert.pem, key.pem)
├── config/                  # Tệp cấu hình tham số teleop và kính Quest 3S (YAML)
├── docs/                    # Tài liệu kiến trúc, đặc tả robot thật và danh mục kiểm tra sim-to-real
├── scripts/                 # Các kịch bản chạy thử, benchmark và kiểm thử tự động
├── teleop/                  # Mã nguồn gói phần mềm chính:
│   ├── kinematics/          # Bộ giải động học ngược DLS IK (agibot_a2_ik.py)
│   ├── retargeting/         # Ánh xạ chuyển động cánh tay và ngón tay OmniHand
│   ├── simulation/          # Môi trường MuJoCo (mujoco_env.py) và máy chủ Web Panel (web_panel.py)
│   ├── tracking/            # Bộ thu dữ liệu bám chuyển động Quest 3S / WebXR WebSocket
│   ├── utils/               # Tiện ích kiểm tra cổng, tạo chứng chỉ SSL, kiểm tra tiến trình
│   └── xr/                  # Kết xuất mô hình kỹ thuật số trong WebXR
├── main.py                  # Điểm khởi chạy thống nhất duy nhất cho toàn bộ hệ thống
└── requirements.txt         # Danh mục thư viện Python bắt buộc
```

---

## 8. Cơ chế an toàn (Safety Layer)

- **Giới hạn khớp (Joint Limits)**: Toàn bộ 14 bậc tự do cánh tay và 12 bậc tự do bàn tay được giám sát và kẹp chặt (`np.clip`) trong phạm vi giới hạn góc cơ học của mô hình URDF/MJCF gốc.
- **Giới hạn vận tốc (Velocity Limits)**: Tốc độ dịch chuyển cổ tay và vận tốc góc khớp bị giới hạn (tối đa 0.25 m/s) nhằm ngăn ngừa chuyển động giật cục hoặc quá tốc.
- **Vùng không gian làm việc (Workspace Limit)**: Giới hạn không gian làm việc an toàn hình hộp chữ nhật phía trước robot: $X \in [0.10, 0.75]\,\text{m}, Y \in [-0.70, 0.70]\,\text{m}, Z \in [0.80, 1.60]\,\text{m}$. Mọi mục tiêu ngoài phạm vi đều bị chặn.
- **Xử lý mất bám theo (Tracking Loss Protection)**: Khi kính mất tín hiệu theo dõi tay hoặc mất kết nối mạng, hệ thống tự động kích hoạt chế độ **Hold Arms** để giữ yên cánh tay, không trôi dạt.
- **E-STOP (Dừng khẩn cấp)**: Nút dừng khẩn cấp phần mềm có thể kích hoạt từ Web UI hoặc phím tắt, lập tức ngắt toàn bộ lệnh điều khiển và ghìm chặt cánh tay.
- **Khóa an toàn robot thật (Hardware Lockout)**: Mọi kết nối tới robot thật đều bị vô hiệu hóa mặc định. Mã điều khiển phần cứng đòi hỏi đồng thời cờ cấu hình tường minh và biến môi trường xác thực an toàn, đảm bảo an toàn tuyệt đối khi thử nghiệm mô phỏng.

---

## 9. Kiểm thử (Verification & Tests)

Chạy các kịch bản kiểm thử tự động (các bài test sẽ tự động khởi động server `main.py` nếu chưa chạy và kiểm tra toàn diện):

```bash
# Kiểm tra đồng bộ khung nhìn 3D Web, truyền dữ liệu WebSocket và IK
python scripts/test_web_sync.py

# Kiểm tra toàn bộ API điều khiển: jog, bàn tay, preset motion, home reset, E-STOP
python scripts/test_panel_api.py

# Kiểm tra độ chính xác của bộ giải Damped Least-Squares IK
python scripts/test_arm_ik.py
```

---

## 10. Xử lý sự cố (Troubleshooting)

- **Cổng 8012 bị bận (`Port 8012 is already in use`)**: `main.py` sẽ tự động hiển thị tiến trình đang chiếm cổng kèm PID. Để giải phóng cổng:
  ```bash
  fuser -k 8012/tcp
  ```
- **Không thấy cửa sổ MuJoCo Native Viewer**:
  - Đảm bảo biến môi trường hiển thị tồn tại (`echo $DISPLAY`). Nếu chạy qua SSH không có X11 forwarding, hệ thống sẽ in cảnh báo và chuyển sang chế độ không cửa sổ; bạn vẫn quan sát được đầy đủ trên Web 3D Viewport (`https://localhost:8012/`).
  - Dùng `--no-viewer` hoặc `--headless` nếu muốn chủ động tắt cửa sổ native viewer.
- **Màn hình kính chỉ hiện lưới xanh**:
  - Kiểm tra xem bạn đã bấm nút **Enter VR** trên trang web trong kính chưa.
  - Xác nhận lệnh ADB reverse đã chạy thành công: `adb reverse tcp:8012 tcp:8012`.
- **Cảnh báo chứng chỉ bảo mật (SSL Certificate Warning)**:
  - Hệ thống sử dụng chứng chỉ tự ký (`certs/cert.pem`). Trên trình duyệt Chrome / Firefox / Quest Browser, chọn **Nâng cao (Advanced)** -> **Tiếp tục truy cập (Proceed / Accept Risk)**.

---

## 11. Lộ trình Sim-to-Real & Tài liệu kỹ thuật

Trước khi triển khai trên robot AGIBOT A2 Ultra thực tế, vui lòng đọc kỹ các tài liệu phân tích và kiểm tra sau:
- **Kiến trúc hệ thống**: [`docs/ARCHITECTURE.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/docs/ARCHITECTURE.md) (luồng dữ liệu WebXR, retargeting, IK, an toàn và các khối chức năng).
- **Phân tích chuyển đổi từ Unitree**: [`docs/AGIBOT_PORTING_ANALYSIS.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/docs/AGIBOT_PORTING_ANALYSIS.md) (so sánh khác biệt kiến trúc giữa xr_teleoperate gốc và bản porting AGIBOT A2).
- **Đặc tả robot thực**: [`docs/A2_REAL_ROBOT_SPEC.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/docs/A2_REAL_ROBOT_SPEC.md) (giao thức điều khiển, thứ tự tên khớp, giới hạn dòng điện và vận tốc).
- **Danh mục kiểm tra an toàn chuyển giao**: [`docs/SIM2REAL_CHECKLIST.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/docs/SIM2REAL_CHECKLIST.md) (các bước chuẩn bị phần cứng, cấu hình mạng và quy trình kiểm tra an toàn trước khi cấp nguồn).
- **Hướng dẫn thiết lập kính Meta Quest 3S**: [`docs/QUEST3S_SETUP.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/docs/QUEST3S_SETUP.md) (chứng chỉ HTTPS, kết nối USB ADB reverse và Wi-Fi).
- **Hướng dẫn mô phỏng MuJoCo**: [`docs/MUJOCO_SETUP.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/docs/MUJOCO_SETUP.md) (cấu trúc mô hình XML, khớp ràng buộc, bộ truyền động và chế độ chạy).
- **Báo cáo kiểm toán & dọn dẹp dự án**: [`docs/CLEANUP_REPORT.md`](file:///home/nguyen/a2_ws/docs/CLEANUP_REPORT.md) (thống kê dọn dẹp, sao lưu `_trash` và lệnh khôi phục).

---

## 12. Quy trình làm việc với Git & Agent (Git Workflow)

Dự án thiết lập quy trình tự động hóa thuận tiện thông qua trợ lý lập trình (Antigravity):
- **`push [ghi chú]`**: Tự động kiểm tra an toàn tệp nhạy cảm/kích thước lớn, chạy kiểm thử chấp nhận (`test_web_sync.py` & `test_panel_api.py`), tạo commit theo chuẩn conventional và đẩy lên nhánh `main`. Dùng `push nhanh` nếu cần bỏ qua bước kiểm thử.
- **`tag vX.Y.Z [thông điệp]`**: Tự động gắn tag phiên bản có chú thích và đẩy lên GitHub. Nếu có từ khóa `release`, hệ thống sẽ đồng thời tạo GitHub Release tự động bằng GitHub CLI (`gh release create`).
- Chi tiết quy tắc kiểm duyệt an toàn xem tại [`AGENTS.md`](file:///home/nguyen/a2_ws/agibot_xr_teleoperate/AGENTS.md).

---

## 13. Giấy phép và ghi công (License & Credits)

- Mã nguồn dự án phát hành theo giấy phép **Apache License 2.0**.
- Kế thừa và phát triển từ kiến trúc của dự án mã nguồn mở `unitreerobotics/xr_teleoperate`.
- Mô hình robot AGIBOT A2 Ultra, tệp mô tả động học MJCF/URDF và các tệp lưới 3D (meshes) thuộc bản quyền của nhà sản xuất **AGIBOT (Agilex Robotics / Zhiyuan Robotics)**; chỉ được sử dụng cho mục đích nghiên cứu, học tập và phát triển tương thích.
