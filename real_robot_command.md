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
# Kiểm tra mã băm SHA256 trước khi gửi:
sha256sum scripts/arm_teleop_rx.py
# Kì vọng: 300f41ebf2c78462e6d34dab842b5b393dd403bfaad0afbb2457de4c64fed6ee

scp scripts/arm_teleop_rx.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/
```

---

## BƯỚC 1: KHỞI ĐỘNG TELEOP SERVER TRÊN LAPTOP

Mở **Terminal 1** trên Laptop:

```bash
cd ~/a2_ws
source venv/bin/activate

# Lệnh tiêu chuẩn cực kỳ đơn giản (mặc định: real-udp, tay cầm Quest bật, speed level 3 = 0.8 rad/s):
python3 main.py

# Tùy chọn thay đổi tốc độ bằng --speed N (1..4, mặc định 3):
# Level 1: 0.4 rad/s (an toàn khởi động)
# Level 2: 0.6 rad/s
# Level 3: 0.8 rad/s (mặc định)
# Level 4: 1.0 rad/s (UNTESTED - sát ngưỡng từ chối của Motion Controller)
# python3 main.py --speed 2

# Chạy mô phỏng MuJoCo thuần:
# python3 main.py --sim
```

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
# Kỳ vọng đúng mã: 300f41ebf2c78462e6d34dab842b5b393dd403bfaad0afbb2457de4c64fed6ee

# 4. Nạp cấu hình ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

# 5. CHUYỂN ACTION THỦ CÔNG (BẮT BUỘC TRƯỚC KHI BẬT LIVE)
python3 SetAction.py --action 402

# 6. Khởi động receiver (mặc định LIVE, IP 192.168.2.75, cổng 5005, thumb swing bật, đếm ngược 3s)
python3 arm_teleop_rx.py

# Tùy chọn chạy dry-run thử nghiệm không xuất xung:
# python3 arm_teleop_rx.py --dry
```

### Bảng Tốc Độ (`--speed N`) - Chu kỳ phát 100 Hz (dt = 0.010 s)
| Level | Max Vel (rad/s) | Max Accel (rad/s²) | Max Step (rad/tick @ 100Hz) | Ghi Chú |
|---|---|---|---|---|
| 1 | 0.40 | 1.00 | 0.004 | Mức an toàn khởi động, làm quen |
| 2 | 0.60 | 1.50 | 0.006 | Tốc độ trung bình |
| 3 | 0.80 | 2.00 | 0.008 | **Mặc định** (mượt mà, không bậc thang) |
| 4 | 1.00 | 2.50 | 0.010 | **UNTESTED**: Cận biên thử nghiệm |

> **Kiểm tra tần số xuất lệnh:** Trên terminal Orin có thể kiểm tra tần số gửi bằng lệnh:
> ```bash
> ros2 topic hz /motion/control/arm_joint_command
> # Kỳ vọng: average rate ~100 Hz (hoặc ~50 Hz nếu chạy cấu hình legacy)
> ```

### Ý Nghĩa Của Log và FAULT
- **Pre-flight Check & DUAL PUBLISHER:** Trước khi bắt đầu xuất lệnh điều khiển, `arm_teleop_rx.py` tự động lắng nghe đo tần số tin nhắn trên `/motion/control/arm_joint_command` và `hand_joint_command` trong 1.5 giây. Nếu phát hiện có node khác đang gửi lệnh (>0 msg/s), receiver sẽ **lập tức từ chối và thoát an toàn**, bảo vệ robot khỏi tình trạng xung đột 2 luồng điều khiển cùng lúc. Trong khi chạy, nếu tần số tin nhắn trên topic vượt quá tốc độ gửi của chính node >10%, receiver sẽ chốt trạng thái `DUAL PUBLISHER`, ngắt xuất lệnh và báo lỗi lên Laptop hiển thị `DUAL PUBLISHER: another node is publishing on robot topics!`.
- **Deviation & Lag (Log-only):** Sai lệch bám đuổi giữa lệnh và robot đo thực **chỉ được ghi log** (max \|sent - act\| hiển thị dạng thông tin trên status). Không bao giờ ngắt lệnh, không phát sinh cảnh báo WARN làm giảm tốc, không có lead limit.
- **Nguyên nhân FAULT còn lại duy nhất:** Chỉ kích hoạt khi vi phạm an toàn phần cứng: vi phạm trần bước nhảy (`max_step > 0.02 rad`), lệnh chứa NaN/inf, xung đột địa chỉ IP phát, hoặc mất kết nối gói tin telemetry (`stale feedback > 300 ms`).
- **LƯU Ý RỦI RO QUAN TRỌNG:** Khi robot di chuyển chậm hơn lệnh mong muốn, cánh tay robot có thể **tiếp tục di chuyển một đoạn ngắn sau khi nhả cò/ly hợp** để bám đuổi kịp vị trí đích (quán tính bám đuổi). **Bắt buộc người thứ hai phải cầm nút E-Stop vật lý trên tay** sẵn sàng dập nút khẩn cấp nếu có bất thường!
- **Trạng thái "Arm not moving reason":** Chỉ hiển thị các lý do thực tế gồm: `not ENGAGED`, `FAULT`, `stale feedback (>200ms)`, `at joint limit`, `target==current`, hoặc `none` khi đang chuyển động bình thường.
- **Self-collision:** Đã được loại bỏ khỏi luồng chặn lệnh và giao diện; không gây khóa cánh tay hay hiện banner cản trở.

> **Các tầng bảo vệ an toàn chuẩn hóa trong `arm_teleop_rx.py`:**
> - **Cổng bảo vệ cuối (Final-Line Guard):** Đúng 1 vị trí publish lệnh duy nhất trên timer 50 Hz. Co giãn đồng tỉ lệ (uniform vector scaling) bảo toàn 100% hướng di chuyển, gia tốc khống chế $\le 3.0\text{ rad/s}^2$, bước nhảy tối đa theo cấp tốc độ: Level 1 $\le 0.008$, Level 2 $\le 0.012$, Level 3 $\le 0.016$, Level 4 $\le 0.020\text{ rad/chu kỳ}$.
> - **Phanh hãm Parabolic rời rạc:** Tự động tính toán khoảng cách phanh trước khi tiếp cận giới hạn khớp (`LIMS`) hoặc vị trí đích, loại bỏ hoàn toàn hiện tượng va chạm biên gây giật nảy.
> - **Xả sạch hàng đợi (Socket Backlog Drain):** Mỗi chu kỳ 20 ms xả sạch buffer và chỉ lấy gói tin mới nhất, triệt tiêu hiện tượng dồn ứ tạo vận tốc đột biến.
> - **Khóa nguồn phát duy nhất (Single Sender Lock):** Khóa chặt phiên phát với IP + SessionID đầu tiên, tự động từ chối và hold an toàn nếu có máy khác cố tình xen lệnh.
> - **Chuyển Action thủ công (Manual SetAction Only):** Đã loại bỏ hoàn toàn auto switch `10 Stand -> 12 Ext Servo` để tránh nguy cơ giật tay đột ngột khi robot chưa đứng vững. Node chỉ kiểm tra read-only `GetAction()` và từ chối bật LIVE nếu chưa ở Action 402/12.
> - **Kiểm tra ổn định khớp trước khi LIVE:** Yêu cầu khớp đứng yên trong 1 giây ($\Delta < 0.003\text{ rad}$) và tự động neo vị trí đo thực trước khi nhận lệnh teleop.
> - **Kiểm tra Topic Bàn Tay (Read-only Hand Publisher Check):** Khởi động đọc số publisher trên `/motion/control/hand_joint_command`. Nếu phát hiện `motion_player` đang chạy, node sẽ cảnh báo nhưng không can thiệp cấu hình robot.
> - **Bảo vệ Bàn Tay (Hand Safety & Clamping):**
>   - Đơn vị: Raw [0..2000] (0 = mở hoàn toàn, 2000 = nắm chặt; Sim: 0.0 rad = mở, 1.30 rad = gập; cùng chiều, không nghịch đảo).
>   - Giới hạn an toàn Teleop: Cắt ngọn cứng ở mức $\le 800$ (tương đương 40% hành trình nắm, mô-men xoắn cố định 1140 ~ 20% lực cực đại).
>   - Ràng buộc kích hoạt: Chỉ gửi lệnh khi phía tay tương ứng ở trạng thái `ENGAGED`. Khi `FAULT`, `hold`, hoặc mất tín hiệu gói tin (>0.400 s), bàn tay tự động trả về 0 (mở an toàn, tuyệt đối không kẹp chặt).
>   - Khung hình chuẩn: Xác thực `frame_id == "AgiHand"`; nếu sai lệch sẽ ngắt phát lệnh tay.
> - **Cơ chế Runaway Guard & Soft Slowdown:** Lệch bám đuổi $\le 0.30$ rad chỉ kích hoạt cảnh báo WARN và hạ tốc độ bước, bảo vệ chống dừng đột ngột. Chỉ khi lệch dạt $> 0.30$ rad liên tục $> 200$ ms mới kích hoạt chốt khóa ngắt an toàn FAULT latch.

---

## BƯỚC 3: QUY TRÌNH THỬ NGHIỆM BÀN TAY AN TOÀN (CÓ NGƯỜI THỨ HAI TRỰC E-STOP)

> **CẢNH BÁO AN TOÀN BẮT BUỘC:** Trong suốt quá trình thử bàn tay, **người thứ hai phải cầm nút E-Stop vật lý** trên tay, sẵn sàng dập nút khẩn cấp ngay lập tức nếu có bất kỳ chuyển động giật hoặc kẹp ngoài ý muốn!

1. **Kiểm tra 1 - Phản hồi Trạng Thái (SYNC Mode, Chưa Gửi Lệnh):**
   - Khi Laptop kết nối tới Orin ở trạng thái `SYNC`, quan sát giao diện 3D: bàn tay robot trong mô phỏng lập tức phản chiếu đúng tư thế thực tế đo từ robot qua topic feedback cổng `5006`.
   - Xác nhận rằng **chưa có bất kỳ lệnh điều khiển bàn tay nào được gửi xuống robot** ở giai đoạn này (lệnh đầu tiên khi chuyển sang `ENGAGED` sẽ bằng chính góc đo thực tế).
2. **Kiểm tra 2 - Khóa Cổng ENGAGED:**
   - Khi chưa bấm `Engage` (hoặc chưa bóp Trigger/Grip trên kính): thử toggler/nắm tay trên giao diện. Xác nhận bàn tay robot thật vẫn đứng yên, lệnh gửi từ chối kẹp.
3. **Kiểm tra 3 - Giữ Trung Tính Khớp Vung Ngón Cái (Thumb Swing):**
   - Xác nhận khớp vung `thumb_0` luôn đứng yên ở vị trí 0 (neutral) nếu không truyền cờ `--enable-thumb-swing`.
4. **Kiểm tra 4 - Thử Nghiệm Nắm Mở Từ Từ (Grasp Test):**
   - Bấm `Engage`. Di chuyển nhẹ ngón tay trên kính hoặc kéo thanh trượt đóng mở bàn tay trên web.
   - Quan sát các ngón tay robot co lại mượt mà, dừng đúng ngưỡng an toàn $\le 800$ raw (chưa tới 1/2 hành trình nắm).
   - Thử thả tay / nhả ly hợp: bàn tay dừng lại hoặc mở về vị trí an toàn.

---

## BƯỚC 4: KÍCH HOẠT VÀ ĐIỀU KHIỂN

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

## BƯỚC 5: DỪNG PHIÊN ĐIỀU KHIỂN AN TOÀN

- Khi kết thúc điều khiển: Bấm `Ctrl + C` trên Terminal SSH của Orin. Node sẽ tự động giữ nguyên vị trí đo thực trong 2 giây trước khi nhả lệnh.
- Nếu có bất thường: **Dập ngay nút E-Stop vật lý trên tay người giám sát** hoặc bấm tổ hợp nút `B + Y` trên tay cầm Quest.
