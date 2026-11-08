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

## BƯỚC 1: KHỞI ĐỘNG TELEOP SERVER TRÊN LAPTOP

Mở **Terminal 1** trên Laptop:

```bash
cd ~/a2_ws
source venv/bin/activate

# 1. Cài đặt token bảo mật môi trường
export TELEOP_TOKEN="a2_teleop"

# 2. Khởi động Teleop Server phát UDP sang Orin
python3 main.py --backend real-udp --controller
```
*(Hệ thống sẽ chạy WebXR server bảo mật tại cổng `8012` và sẵn sàng stream UDP sang `192.168.2.50:5005`)*.

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

# 3. Kích hoạt chế độ Servo ngoài cho cánh tay
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
python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005
```

> **Mẹo chỉnh tốc độ:**
> - Mặc định tốc độ an toàn là `--max-step 0.01` ($\approx 30^\circ/\text{s}$).
> - Nếu muốn robot phản hồi nhanh hơn, thêm `--max-step 0.02` ($\approx 60^\circ/\text{s}$):
>   ```bash
>   python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005 --max-step 0.02
>   ```

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
