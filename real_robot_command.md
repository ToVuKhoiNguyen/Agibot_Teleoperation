# Hướng Dẫn Vận Hành Teleop Robot Thật Bằng Kính Quest 3

Tài liệu hướng dẫn từng bước chi tiết từ lúc vừa bật máy tính, áp dụng cho hệ thống **AGIBOT A2 Ultra**.

---

## 0. Thông Số Mạng Cố Định
- **Máy trạm Teleop (Laptop):** `192.168.2.75`
- **Máy tính nhúng Robot (Orin):** `192.168.2.50` *(SSH user: `agi`, mật khẩu: `1`)*
- **Địa chỉ giao diện Web:** `https://192.168.2.75:8012`
- **Cổng UDP nhận lệnh từ Laptop sang Orin:** `5005`
- **Cổng UDP phản hồi feedback từ Orin về Laptop:** `5006`

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

# Khởi động Teleop Server phát UDP sang Orin (mặc định headless, tắt mô phỏng MuJoCo step)
# Chế độ điều khiển tay cầm Quest (khuyên dùng):
python3 main.py --backend real-udp --controller --i-understand-hardware-risk

# Hoặc chế độ điều khiển chuột/gizmo 3D trên Web:
# python3 main.py --backend real-udp --i-understand-hardware-risk

# Nếu muốn mở kèm cửa sổ viewer đồ họa MuJoCo desktop, thêm cờ --viewer:
# python3 main.py --backend real-udp --controller --viewer --i-understand-hardware-risk
```
*(Khi chạy `--backend real-udp`, hệ thống kết nối UDP 50 Hz monotonic với Orin, kích hoạt khóa single-sender `/tmp/a2_teleop.lock` (theo dõi PID sống), tự động đồng bộ tư thế robot đo thực khi kết nối và bảo vệ lệch khớp với Mirror Limiter)*.

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
# Kỳ vọng đúng mã: 35f64b57af38dc01aeb7ba7d24636cacf74332da400e45501cda0c6b7dcb0c37

# 4. Nạp cấu hình ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

# 5. CHUYỂN ACTION THỦ CÔNG (BẮT BUỘC TRƯỚC KHI BẬT LIVE)
# Vì lý do an toàn tuyệt đối, arm_teleop_rx.py đã loại bỏ tính năng tự động chuyển action.
# Hãy dùng script của vendor để chuyển sang Action Ext Joint Servo (Action 402 / 12):
python3 SetAction.py --action 402
# (Hoặc: python3 SetAction.py --action 12 tùy phiên bản SDK vendor)

# 6. Kích hoạt nhận lệnh điều khiển động cơ thật (LIVE)
# Chế độ Tiêu Chuẩn (Profile Normal: 0.3 rad/s - êm ru, đã kiểm định an toàn):
python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005 --speed-profile normal

# Hoặc chế độ Rất Chậm để làm quen (Profile Slow: 0.2 rad/s):
# python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip 192.168.2.75 --port 5005 --speed-profile slow
```

> **Các tầng bảo vệ an toàn chuẩn hóa trong `arm_teleop_rx.py`:**
> - **Cổng bảo vệ cuối (Final-Line Guard):** Đúng 1 vị trí publish lệnh duy nhất trên timer 50 Hz. Co giãn đồng tỉ lệ (uniform vector scaling) bảo toàn 100% hướng di chuyển, gia tốc khống chế $\le 2.0\text{ rad/s}^2$ (trần cứng $\le 3.0\text{ rad/s}^2$), bước nhảy $\le 0.010\text{ rad/chu kỳ}$ (trần cứng $\le 0.016\text{ rad}$).
> - **Phanh hãm Parabolic rời rạc:** Tự động tính toán khoảng cách phanh trước khi tiếp cận giới hạn khớp (`LIMS`) hoặc vị trí đích, loại bỏ hoàn toàn hiện tượng va chạm biên gây giật nảy.
> - **Xả sạch hàng đợi (Socket Backlog Drain):** Mỗi chu kỳ 20 ms xả sạch buffer và chỉ lấy gói tin mới nhất, triệt tiêu hiện tượng dồn ứ tạo vận tốc đột biến.
> - **Khóa nguồn phát duy nhất (Single Sender Lock):** Khóa chặt phiên phát với IP + SessionID đầu tiên, tự động từ chối và hold an toàn nếu có máy khác cố tình xen lệnh.
> - **Chuyển Action thủ công (Manual SetAction Only):** Đã loại bỏ hoàn toàn auto switch `10 Stand -> 12 Ext Servo` để tránh nguy cơ giật tay đột ngột khi robot chưa đứng vững. Node chỉ kiểm tra read-only `GetAction()` và từ chối bật LIVE nếu chưa ở Action 402/12.
> - **Kiểm tra ổn định khớp trước khi LIVE:** Yêu cầu khớp đứng yên trong 1 giây ($\Delta < 0.003\text{ rad}$) và không có publisher nào khác trên topic `/motion/control/arm_joint_command` trước khi nhận lệnh teleop.
> - **Mô hình độ trễ bậc 1 (First-order Lag Gate):** Hỗ trợ cờ `--gate-mode lagged` để tính toán sai số động học theo hằng số thời gian $\tau$ đã nhận diện thực tế ($\tau \approx 0.138\text{ s}$), ngăn ngừa ngắt nhầm do quán tính cơ cấu.

---

## BƯỚC 3: KÍCH HOẠT VÀ ĐIỀU KHIỂN

### Cách A: Điều khiển bằng Kính Meta Quest 3 (VR)
1. Đeo kính Meta Quest (đã kết nối cùng mạng Wifi với Laptop).
2. Mở trình duyệt Browser trên kính, truy cập: `https://192.168.2.75:8012`
   *(Nếu thấy cảnh báo chứng chỉ SSL tự ký, chọn **Advanced** $\rightarrow$ **Proceed to 192.168.2.75**)*.
3. Bấm **ENTER VR**.
4. **Thao tác tay cầm:**
   - Quan sát 2 tay robot **đứng yên hoàn toàn** tại vị trí ban đầu (Anchored pose).
   - Nhấn giữ nút **Trigger** (Cò ngón trỏ) hoặc **Grip** (Cò cạnh bàn tay) để bóp ly hợp (Clutch) và di chuyển: hai cánh tay robot thật sẽ bám theo chuyển động tay của bạn trong không gian!

### Cách B: Điều khiển bằng Chuột / 3D Gizmo trên trình duyệt Laptop
1. Mở trình duyệt trên máy trạm: `https://localhost:8012`
2. Kiểm tra thanh trạng thái hiển thị `READY (NOT ENGAGED)`.
3. Bấm nút **Engage** (hoặc nút **Home** / **T-Pose** / kéo Gizmo $\pm X, \pm Y, \pm Z$).
4. Robot sẽ chuyển sang trạng thái `ENGAGED` và bám theo mục tiêu 1:1 mượt mà.
5. Nếu gặp chướng ngại hoặc chạm biên vật lý gây FAULT: bấm nút **Clear Fault** trên banner để đưa cánh tay về trạng thái an toàn `READY`.

---

## BƯỚC 4: DỪNG PHIÊN ĐIỀU KHIỂN AN TOÀN

- Khi kết thúc điều khiển: Bấm `Ctrl + C` trên Terminal SSH của Orin. Node sẽ tự động giữ nguyên vị trí đo thực trong 2 giây trước khi nhả lệnh.
- Nếu có bất thường: **Dập ngay nút E-Stop vật lý trên tay người giám sát** hoặc bấm tổ hợp nút `B + Y` trên tay cầm Quest.
