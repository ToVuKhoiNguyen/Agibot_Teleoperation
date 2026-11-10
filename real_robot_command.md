# Hướng Dẫn Vận Hành Teleop Robot Thật Bằng Kính Quest 3

Tài liệu hướng dẫn từng bước chi tiết từ lúc vừa bật máy tính, áp dụng cho hệ thống **AGIBOT A2 Ultra**.

---

## 0. Thông Số Mạng Cố Định
- **Máy trạm Teleop (Laptop):** `192.168.2.75`
- **Máy tính nhúng Robot (Orin):** `192.168.2.50` *(SSH user: `agi`, mật khẩu: `1`)*
- **Địa chỉ giao diện Web:** `https://192.168.2.75:8012`
- **Cổng UDP stream góc khớp:** `5005`
- **Mã bảo mật phiên:** `export TELEOP_TOKEN="a2_teleop"`

---

## BƯỚC 0: ĐỒNG BỘ CODE SANG ROBOT (NẾU CÓ CẬP NHẬT)

Mở terminal trên Laptop và chạy lệnh:
```bash
scp scripts/arm_teleop_rx.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/
```

---

## BƯỚC 1: KHỞI ĐỘNG TELEOP SERVER TRÊN LAPTOP

Mở **Terminal 1** trên Laptop:

```bash
cd ~/a2_ws
source venv/bin/activate

# 1. Cài đặt token bảo mật môi trường
export TELEOP_TOKEN="a2_teleop"

# 2. Khởi động Teleop Server phát UDP sang Orin (mặc định headless, tắt hoàn toàn mô phỏng MuJoCo)
# Chế độ mượt mà như arm.py (khuyên dùng):
python3 main.py --backend real-udp --controller --real-max-vel 1.0 --i-understand-hardware-risk

# Hoặc chế độ tốc độ cao:
# python3 main.py --backend real-udp --controller --real-max-vel 1.8 --i-accept-higher-speed --i-understand-hardware-risk
```
*(Khi chạy `--backend real-udp`, hệ thống tự động tắt hàm mô phỏng vật lý `env.step()`, ngắt nạp lệnh motor ảo MuJoCo và chạy headless để tiết kiệm tối đa CPU laptop. Thêm cờ `--viewer` nếu bạn muốn mở cửa sổ đồ họa desktop)*.
*(Hệ thống sẽ chạy WebXR server bảo mật tại cổng `8012` và stream UDP mượt mà sang `192.168.2.50:5005`)*.

---

## BƯỚC 2: KHỞI ĐỘNG ROBOT RECEIVER TRÊN ORIN

Mở **Terminal 2** trên Laptop và SSH vào Orin:

```bash
# 1. SSH vào robot
ssh agi@192.168.2.50
# Mật khẩu: 1

# 2. Nạp môi trường AGI và tắt motion player
aima em load-env
bash /agibot/data/home/agi/Desktop/motion_player/disable_motion_player.sh

# 3. Vào thư mục mc và chọn Action 12 (hoặc để script tự gọi RPC)
cd /agibot/data/home/agi/Desktop/mc
python3 SetAction.py
# (Chọn: McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO)

# 4. Nạp cấu hình ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

# 5. Kích hoạt nhận lệnh điều khiển động cơ thật (LIVE)
export TELEOP_TOKEN="a2_teleop"

# Chế độ mượt mà như arm.py (khuyên dùng - êm ru, lọc sạch rung giật Wi-Fi):
python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005 --max-step 0.02 --max-acc 5.0 --omega 12.0

# Hoặc chế độ mặc định (nhanh & nhạy):
# python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005
```

> **Các tính năng tự động thông minh trong `arm_teleop_rx.py`:**
> - **Bộ lọc dao động tắt dần bậc 2 (Critically Damped Follower):** $\ddot{q} = \omega^2 (q_{tgt} - q) - 2\omega \dot{q}$ với giới hạn gia tốc khắt khe ($|a| \le 5.0\text{ rad/s}^2$). Lọc phẳng hoàn toàn hiện tượng bậc thang, chuyển động êm ái như `arm.py`.
> - **Tự động SetAction:** Script tự gửi lệnh RPC nội bộ chuyển Action 10 (`McAction_RL_JOINT_DEFAULT`) $\rightarrow$ Action 12 (`McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`), bạn không cần chạy thủ công `python3 SetAction.py` nữa!
> - **Tự động Auto-Homing:** Ngay khi chạy, robot tự đo góc khớp thực tế và lướt êm ái về vị trí HOME trong 2.5–5s (in chữ `HOMED` khi hoàn thành).
> - **Deadband chống rung tay ($0.003\text{ rad} \approx 0.17^\circ$):** Triệt tiêu hoàn toàn hiện tượng rung lắc/wobble khi tay dừng ở đích.

> **Mẹo tùy chỉnh độ mượt & tốc độ:**
> - `--max-step 0.02` đến `0.035`: Vận tốc tối đa ($1.0 - 1.75\text{ rad/s} \approx 57^\circ - 100^\circ/\text{s}$).
> - `--max-acc 5.0` (êm ru) hoặc `8.0` (nhanh hơn).
> - `--omega 10.0` đến `14.0`: Độ nhạy phản hồi (tương đương $\tau \approx 70-100\text{ ms}$).
> - `--deadband 0.003`: Ngưỡng lọc vi rung tay.

---

## BƯỚC 3: ĐEO KÍNH QUEST & BẮT ĐẦU ĐIỀU KHIỂN

1. Đeo kính Meta Quest (đã kết nối cùng mạng Wifi với Laptop).
2. Mở trình duyệt Browser trên kính, truy cập:
   `https://192.168.2.75:8012`
   *(Nếu thấy cảnh báo chứng chỉ SSL tự ký, chọn **Advanced** $\rightarrow$ **Proceed to 192.168.2.75**)*.
3. Bấm **ENTER VR**.
4. **Thao tác tay cầm:**
   - Quan sát 2 tay robot **đứng yên hoàn toàn** tại vị trí ban đầu (Anchored pose).
   - Nhấn giữ nút **Trigger** (Cò ngón trỏ) hoặc **Grip** (Cò cạnh bàn tay) để bóp ly hợp (Clutch) và di chuyển: hai cánh tay robot thật sẽ bám theo chuyển động tay của bạn trong không gian!

---

## BƯỚC 4: DỪNG PHIÊN ĐIỀU KHIỂN AN TOÀN

- Khi kết thúc điều khiển: Bấm `Ctrl + C` trên Terminal SSH của Orin. Node sẽ tự động giữ nguyên vị trí đo thực trong 2 giây trước khi nhả lệnh.
- Nếu có bất thường: **Dập ngay nút E-Stop vật lý trên tay người giám sát** hoặc bấm tổ hợp nút `B + Y` trên tay cầm Quest.
