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
# Chế độ điều khiển tiêu chuẩn (khuyên dùng):
python3 main.py --backend real-udp --controller --i-understand-hardware-risk

# Nếu muốn mở cửa sổ hiển thị đồ họa desktop, thêm cờ --viewer:
# python3 main.py --backend real-udp --controller --viewer --i-understand-hardware-risk
```
*(Khi chạy `--backend real-udp`, hệ thống tự động tắt hàm mô phỏng vật lý `env.step()`, ngắt nạp lệnh motor ảo MuJoCo và chạy headless để tiết kiệm tối đa CPU laptop. Laptop tự động kích hoạt bộ đệm phát 50 Hz monotonic, khóa single-sender `/tmp/a2_teleop.lock` và kiểm tra phiên bản protocol v2)*.

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

# 3. Vào thư mục mc và kiểm tra mã băm SHA256 file receiver
cd /agibot/data/home/agi/Desktop/mc
sha256sum arm_teleop_rx.py
# Kỳ vọng đúng mã: 5dd1cce57d748c1b65b6f16d5aba76231e668b6d66a30264d4d2e7bca331274a

# 4. Nạp cấu hình ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

# 5. Cài đặt token bảo mật phiên
export TELEOP_TOKEN="a2_teleop"

# (Khuyên dùng) Thử nghiệm dry-run trước khi bật động cơ thật:
# python3 arm_teleop_rx.py --dry --allow-ip 192.168.2.75 --port 5005

# 6. Kích hoạt nhận lệnh điều khiển động cơ thật (LIVE)
# Chế độ Tiêu Chuẩn (Profile Normal: 0.3 rad/s - êm ru, đã kiểm định an toàn):
python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005 --speed-profile normal

# Hoặc chế độ Rất Chậm để làm quen (Profile Slow: 0.2 rad/s):
# python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005 --speed-profile slow
```

> **Các tầng bảo vệ an toàn chuẩn hóa trong `arm_teleop_rx.py`:**
> - **Cổng bảo vệ cuối (Final-Line Guard):** Đúng 1 vị trí publish lệnh duy nhất trên timer 50 Hz. Co giãn đồng tỉ lệ (uniform vector scaling) bảo toàn 100% hướng di chuyển, gia tốc khống chế $\le 1.05\text{ rad/s}^2$, bước nhảy tối đa $\le 0.010\text{ rad/chu kỳ}$.
> - **Phanh hãm Parabolic rời rạc:** Tự động tính toán khoảng cách phanh trước khi tiếp cận giới hạn khớp (`LIMS`) hoặc vị trí đích, loại bỏ hoàn toàn hiện tượng va chạm biên gây giật nảy.
> - **Xả sạch hàng đợi (Socket Backlog Drain):** Mỗi chu kỳ 20 ms xả sạch buffer và chỉ lấy gói tin mới nhất, triệt tiêu hiện tượng dồn ứ tạo vận tốc đột biến.
> - **Khóa nguồn phát duy nhất (Single Sender Lock):** Khóa chặt phiên phát với IP + SessionID đầu tiên, tự động từ chối và hold an toàn nếu có máy khác cố tình xen lệnh.
> - **Tự động SetAction:** Tự gửi lệnh RPC nội bộ chuyển Action 10 $\rightarrow$ Action 12 (`McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`), không cần thao tác `SetAction.py` thủ công.
> - **Tự động Auto-Homing:** Robot tự đo góc thực tế và di chuyển êm dịu về vị trí HOME trong 2.5–5s (báo chữ `HOMED` khi sẵn sàng nhận teleop).

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
