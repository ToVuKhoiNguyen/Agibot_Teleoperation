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

# 2. Khởi động Teleop Server phát UDP sang Orin (mở khóa vận tốc mượt)
python3 main.py --backend real-udp --controller --real-max-vel 1.8 --i-accept-higher-speed --i-understand-hardware-risk
```
*(Thêm `--headless` nếu muốn tắt hoàn toàn cửa sổ MuJoCo)*.
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

# 5. Kích hoạt nhận lệnh điều khiển động cơ thật (LIVE) - Mặc định Bộ 2: Nhanh & Nhạy
export TELEOP_TOKEN="a2_teleop"
python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005
```

> **Các tính năng tự động thông minh trong `arm_teleop_rx.py`:**
> - **Cấu hình mặc định (Bộ 2: Nhanh & Nhạy):** `--max-step 0.035` ($1.75\text{ rad/s}$), `--alpha 0.30` (bám sát tay, giảm thiểu delay), `--deadband 0.003` (lọc rung tay). Không cần gõ thêm cờ nào!
> - **Tự động SetAction:** Script tự gửi lệnh RPC nội bộ chuyển Action 10 (`McAction_RL_JOINT_DEFAULT`) $\rightarrow$ Action 12 (`McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`), bạn không cần chạy thủ công `python3 SetAction.py` nữa!
> - **Tự động Auto-Homing:** Ngay khi chạy, robot tự đo góc khớp và lướt êm ái về vị trí HOME trong 2.5–5s (in chữ `HOMED` khi hoàn thành).
> - **Bộ lọc Exponential Ease-Out:** Triệt tiêu hiện tượng giật nấc/bậc thang, tự động giảm tốc êm khi tiếp cận mục tiêu.
> - **Deadband chống rung tay:** Khóa cứng khớp khi tay dừng ở đích, không bao giờ bị đu đưa hay rung lắc.

> **Mẹo tùy chỉnh độ mượt & tốc độ:**
> - `--max-step 0.025` đến `0.035`: Tốc độ di chuyển tối đa của khớp ($\approx 70^\circ - 100^\circ/\text{s}$).
> - `--alpha 0.20` (siêu mượt, êm ái) hoặc `--alpha 0.30` (nhạy, bám sát tay hơn).
> - `--deadband 0.003`: Ngưỡng lọc rung tay vi mô ($\approx 0.17^\circ$).

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
