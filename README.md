# AGIBOT A2 Ultra - Hệ thống Điều khiển Từ xa XR & Mô phỏng Song sinh Kỹ thuật số (Digital Twin)

Hệ thống điều khiển từ xa (Teleoperation) độ trễ thấp và song sinh số thời gian thực cho robot hình người hai tay AGIBOT A2 Ultra kết hợp bàn tay khéo léo OmniHand (5 ngón).

---

## 1. Giới thiệu

- **Mục tiêu**: Cung cấp giải pháp teleoperation trực quan qua kính thực tế ảo Meta Quest 3S (WebXR) hoặc giao diện web 3D trên PC, giải thuật động học ngược (Mink / Damped Least-Squares IK), retargeting cánh tay và bàn tay khéo léo, đồng bộ song sinh số 3D với tần số điều khiển cố định 200 Hz.
- **An toàn cốt lõi**: **Chỉ chạy mô phỏng vật lý (MuJoCo) theo mặc định**. Phần cứng robot thật hoàn toàn bị khóa ở mức logic và cấu hình. Tích hợp lớp phòng ngừa tự va chạm (Self-collision prevention) thời gian thực.

---

## 2. Yêu cầu hệ thống và cài đặt

- **Hệ điều hành**: Linux (Ubuntu 22.04 / 24.04 LTS).
- **Python**: Khuyến nghị Python 3.10 - 3.12 trong môi trường ảo `venv`.
- **Phụ thuộc**: `pip install -r requirements.txt` (MuJoCo, Mink, AioHTTP, PyYAML, NumPy, SciPy, Trimesh, Ruckig...).
- **ADB**: Cài đặt Android Debug Bridge (`sudo apt install adb`) để kết nối có dây USB với kính Quest 3S.

```bash
cd ~/a2_ws
source venv/bin/activate
pip install -r requirements.txt
```

---

## 3. Chạy nhanh (Quick Start)

Khởi động toàn bộ hệ thống (MuJoCo physics, Web Panel HTTPS/WSS, WebXR server):

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
| `--ik` | `str` | `mink` | Động cơ động học ngược: `mink` (mặc định), `dls`, hoặc `pyroki` |
| `--self-collision` | `str` | `on` | Bật/tắt cổng an toàn chống tự va chạm: `on` hoặc `off` |
| `--collision-avoidance` | Cờ | `False` | Bật ràng buộc tránh va chạm của bộ giải Mink QP |
| `--no-adb` | Cờ | `False` | Tắt tự động phát hiện ADB và cấu hình reverse port |
| `--no-quest-launch` | Cờ | `False` | Không tự động kích hoạt trình duyệt Quest Browser trên kính |

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
- **3D Combined Gizmo**: Hiển thị đồng thời mũi tên tịnh tiến, vòng xoay và mặt phẳng điều khiển trực tiếp trên 2 cổ tay robot. Hỗ trợ phím tắt `W` (chỉ dịch chuyển), `E` (chỉ xoay), `R` (hiển thị cả hai - mặc định), `Q` (chuyển đổi World / Local).
- **An toàn khẩn cấp & Giám sát**:
  - Nút **E-STOP** đỏ lớn: Dừng khẩn cấp toàn bộ chuyển động, ngắt lệnh điều khiển.
  - Nút **Clear E-STOP**: Hủy trạng thái dừng khẩn cấp để tiếp tục điều khiển.
  - Nút **Hold Arms**: Khóa cứng vị trí hiện tại của cánh tay.
  - Nút **Reset**: Đưa toàn bộ robot về keyframe home ban đầu.
  - Hiển thị khoảng cách tự va chạm nhỏ nhất thời gian thực (`min_distance_mm`) và cảnh báo khi bị chặn va chạm.

---

## 6. Kết nối Quest 3S qua USB & WebXR (Tự động hóa hoàn toàn)

Chỉ cần **cắm cáp USB-C hỗ trợ truyền dữ liệu (data cable)**, bấm **Allow USB debugging** trong kính và chạy `python main.py`. Hệ thống sẽ tự động cấu hình ADB reverse và mở trang WebXR trên kính.

1. **Quy trình kết nối nhanh**:
   - **Bước 1**: Cắm cáp USB-C kết nối kính Meta Quest 3S vào máy tính (lưu ý dùng cáp hỗ trợ truyền dữ liệu, không chỉ sạc).
   - **Bước 2**: Đeo kính và bấm **Allow** (Cho phép gỡ lỗi USB).
   - **Bước 3**: Trên máy tính, khởi động hệ thống:
     ```bash
     cd ~/a2_ws
     source venv/bin/activate
     python main.py
     ```
     `main.py` sẽ tự động phát hiện kính qua ADB, thiết lập `adb reverse tcp:8012 tcp:8012` và tự động mở trình duyệt Quest Browser tới trang **`https://localhost:8012/vr`**.
   - **Bước 4**: Trong kính, chấp nhận chứng chỉ SSL tự ký lần đầu tiên (**Advanced** -> **Proceed to localhost**), sau đó bấm nút to **ENTER VR** (hoặc bóp cò trigger).
2. **Kiểm tra chẩn đoán kết nối**:
   - Chạy công cụ kiểm tra tự động:
     ```bash
     python scripts/quest_doctor.py
     ```
3. **Thao tác tay cầm Quest trong không gian VR (Controller Mapping)**:

| Nút / Thao tác | Chức năng điều khiển |
| :--- | :--- |
| **Cò hông (Grip)** | Chỉ tịnh tiến (X,Y,Z theo clutch delta, xoay khóa) |
| **Cò trước (Trigger)** | Chỉ xoay (roll/pitch/yaw từ anchor, vị trí khóa) |
| **Cả hai (Grip + Trigger)** | 6D đầy đủ (vừa tịnh tiến vừa xoay cổ tay) |
| **Không bóp cò** | Khóa cứng cánh tay; tự động cập nhật lại anchor từ tư thế thực tế hiện tại (không nhảy bước) |
| **Analog Trigger** | Điều khiển độ khép mở của 5 ngón tay OmniHand |
| **Joystick Y (trục dọc)** | Nắm/xòe ngón tay theo vận tốc (đẩy = nắm, kéo = xòe, thả = giữ) |
| **Joystick X (trục ngang)** | Đổi kiểu gắp (cylinder, pinch, tripod, point, thumbs-up) |
| **Click Joystick** | Bung xòe khẩn cấp (Panic Open, closure -> 0) |
| **A (phải) / X (trái)** | Reset quả cầu mục tiêu về đúng vị trí cổ tay thực tế hiện tại |
| **B (phải) / Y (trái)** | Bật/tắt chế độ Hold (giữ nguyên vị trí cánh tay) |
| **Nút Menu** | E-STOP tức thì |
| **Tổ hợp B + Y giữ > 0.5 s** | Kích hoạt E-STOP khẩn cấp |
| **An toàn watchdog** | Mất tín hiệu kính > 350 ms: tự động nhả clutch và khóa tư thế tay robot |

---

## 7. Cấu trúc thư mục

```
agibot_xr_teleoperate/
├── assets/                  # Mô hình robot AGIBOT A2 Ultra (MJCF, URDF, meshes STL và web_meshes)
├── certs/                   # Chứng chỉ SSL HTTPS tự sinh (cert.pem, key.pem)
├── config/                  # Tệp cấu hình tham số teleop, kính Quest 3S và safety (YAML)
├── docs/                    # Tài liệu kiến trúc, đặc tả robot thật, vendor manuals và danh mục kiểm tra sim-to-real
│   └── vendor/              # Tài liệu manual PDF và dev kit chính hãng của AgiBot
├── scripts/                 # Các kịch bản chạy thử, benchmark và kiểm thử tự động
├── teleop/                  # Mã nguồn gói phần mềm chính:
│   ├── kinematics/          # Bộ giải động học ngược IK (Mink IK, DLS IK, Pyroki IK) & bộ lọc quỹ đạo Ruckig
│   ├── retargeting/         # Ánh xạ chuyển động cánh tay và bàn tay khéo léo OmniHand (5 ngón)
│   ├── simulation/          # Môi trường MuJoCo (mujoco_env.py) và máy chủ Web Panel (web_panel.py)
│   ├── tracking/            # Bộ thu dữ liệu bám chuyển động Quest 3S / WebXR WebSocket
│   ├── utils/               # Cổng an toàn tự va chạm (collision_checker.py), chứng chỉ SSL, kiểm tra tiến trình
│   └── xr/                  # Kết xuất mô hình kỹ thuật số trong WebXR
├── tests/                   # Bộ kiểm thử đơn vị (Unit tests)
├── AGENTS.md                # Quy chuẩn và chỉ dẫn an toàn dành cho AI Agent
├── LICENSE                  # Giấy phép mã nguồn mở
├── main.py                  # Điểm khởi chạy thống nhất duy nhất cho toàn bộ hệ thống
├── requirements.txt         # Danh mục thư viện Python bắt buộc
└── requirements-ros.txt     # Danh mục gói phụ thuộc ROS 2 (cho robot thật)
```

---

## 8. Cơ chế an toàn (Safety Layer)

- **Phòng ngừa tự va chạm (Self-Collision Prevention Gate)**: Tích hợp `CollisionChecker` sử dụng các geom va chạm đơn giản hóa (capsule, sphere, box) chạy song song với `mujoco.mj_geomDistance`. Kiểm tra va chạm đa điểm giữa tay-thân, tay-đầu, tay-tay với ngưỡng an toàn `5 mm`. Khi phát hiện va chạm, thuật toán tự động chiếu trượt tiếp tuyến (tangent projection) hoặc chặn chuyển động không cho xuyên thấu.
- **Bộ lọc quỹ đạo chuyển động mượt mà (Smooth Trajectory Filters)**: Áp dụng `CriticallyDampedFollower` (tần số riêng 30 Hz) cho mục tiêu gizmo và `JointTrajectoryFilter` (Ruckig OTG bậc 3) cho góc khớp, loại bỏ hoàn toàn hiện tượng rung giật khi bám theo chuột/gizmo.
- **Giới hạn khớp (Joint Limits)**: Toàn bộ 14 bậc tự do cánh tay và 12 bậc tự do bàn tay được giám sát và kẹp chặt (`np.clip`) trong phạm vi giới hạn góc cơ học của mô hình URDF/MJCF gốc.
- **Giới hạn vận tốc & gia tốc (Velocity & Acceleration Limits)**: Vận tốc góc khớp và gia tốc được giới hạn theo thông số danh định của robot, ngăn ngừa giật cục cơ học.
- **Vùng không gian làm việc (Workspace Limit)**: Giới hạn không gian làm việc an toàn hình hộp chữ nhật: $X \in [-0.20, 0.90]\,\text{m}, Y \in [-0.90, 0.90]\,\text{m}, Z \in [0.50, 1.80]\,\text{m}$. Mọi mục tiêu ngoài phạm vi đều bị kẹp tự động.
- **Xử lý mất bám theo (Watchdog Protection)**: Khi kính mất tín hiệu theo dõi tay hoặc timeout > 350 ms, hệ thống tự động ngắt clutch và khóa tư thế hiện tại.
- **E-STOP (Dừng khẩn cấp)**: Có thể kích hoạt tức thời từ Web UI, phím bấm, hoặc tay cầm Quest (B+Y giữ 0.5s), lập tức ngắt toàn bộ lệnh điều khiển và ghìm chặt cánh tay.

---

## 9. Kiểm thử tự động (Verification & Tests)

Chạy các kịch bản kiểm thử chấp nhận tự động (các bài test sẽ tự động khởi động server `main.py` nếu chưa chạy và kiểm tra toàn diện):

```bash
cd ~/a2_ws
source venv/bin/activate

# 1. Kiểm tra toàn bộ API điều khiển: jog, bàn tay, preset motion, home reset, E-STOP
python scripts/test_panel_api.py

# 2. Kiểm tra phòng ngừa tự va chạm (Torso, Head, Hand-vs-Hand, Latency benchmark)
python scripts/test_self_collision.py

# 3. Kiểm tra đồng bộ khung nhìn 3D Web, truyền dữ liệu WebSocket và IK
python scripts/test_web_sync.py

# 4. Kiểm tra điều khiển tay cầm kính Quest 3S và các trục joystick
python scripts/test_xr_controllers.py
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

## 11. Tài liệu kỹ thuật chi tiết

- **Kiến trúc hệ thống**: [`docs/ARCHITECTURE.md`](file:///home/nguyen/a2_ws/docs/ARCHITECTURE.md) (luồng dữ liệu WebXR, retargeting, IK, an toàn và các khối chức năng).
- **Đặc tả robot thực**: [`docs/A2_REAL_ROBOT_SPEC.md`](file:///home/nguyen/a2_ws/docs/A2_REAL_ROBOT_SPEC.md) (giao thức điều khiển, thứ tự tên khớp, giới hạn dòng điện và vận tốc).
- **Danh mục kiểm tra an toàn chuyển giao**: [`docs/SIM2REAL_CHECKLIST.md`](file:///home/nguyen/a2_ws/docs/SIM2REAL_CHECKLIST.md) (các bước chuẩn bị phần cứng, cấu hình mạng và quy trình kiểm tra an toàn trước khi cấp nguồn).
- **Hướng dẫn thiết lập kính Meta Quest 3S**: [`docs/QUEST3S_SETUP.md`](file:///home/nguyen/a2_ws/docs/QUEST3S_SETUP.md) (chứng chỉ HTTPS, kết nối USB ADB reverse và Wi-Fi).
- **Hướng dẫn mô phỏng MuJoCo**: [`docs/MUJOCO_SETUP.md`](file:///home/nguyen/a2_ws/docs/MUJOCO_SETUP.md) (cấu trúc mô hình XML, khớp ràng buộc, bộ truyền động và chế độ chạy).
- **Tài liệu PDF gốc từ AgiBot**: Nằm tại [`docs/vendor/`](file:///home/nguyen/a2_ws/docs/vendor/) gồm cẩm nang vận hành robot (`agibot_a2_manual.pdf`) và hướng dẫn lập trình SDK (`aimdk_a2_dev.pdf`).

---

## 12. Quy trình làm việc với Git & Agent (Git Workflow)

Dự án thiết lập quy trình tự động hóa thuận tiện thông qua trợ lý lập trình (Antigravity):
- **`push [ghi chú]`**: Tự động kiểm tra an toàn tệp nhạy cảm/kích thước lớn, chạy kiểm thử chấp nhận (`test_web_sync.py` & `test_panel_api.py`), tạo commit theo chuẩn conventional và đẩy lên nhánh `main`. Dùng `push nhanh` nếu cần bỏ qua bước kiểm thử.
- **`tag vX.Y.Z [thông điệp]`**: Tự động gắn tag phiên bản có chú thích và đẩy lên GitHub. Nếu có từ khóa `release`, hệ thống sẽ đồng thời tạo GitHub Release tự động bằng GitHub CLI (`gh release create`).
- Chi tiết quy tắc kiểm duyệt an toàn xem tại [`AGENTS.md`](file:///home/nguyen/a2_ws/AGENTS.md).
