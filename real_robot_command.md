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
sha256sum scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py
# Kì vọng arm_teleop_rx.py: 2d66854067335775eb73dc3085a6e67e5a4e2b74db3c29a41e155305367f5ba9

# Đồng bộ sang Orin (cả 4 tệp độc lập vào thư mục mc):
scp scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/
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
# Kỳ vọng đúng mã: 2d66854067335775eb73dc3085a6e67e5a4e2b74db3c29a41e155305367f5ba9

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
- **Tự va chạm giáp bọc (Armor Collision Check & Soft Block):** Thay thế cơ chế proxy collision cũ. Mọi hình học va chạm URDF được bao bọc lớp giáp dày 1 cm (`shell_m: 0.010` m, ngưỡng tiếp xúc $2 \times \text{shell\_m} = 0.020$ m). Khi khoảng cách bề mặt $\le 20$ mm, bộ giới hạn lệnh (limiter) kích hoạt Soft Block: giữ nguyên tư thế hợp lệ cuối cùng với $v=0$ (PLAN bit-identical), không gây FAULT, không ngắt kết nối. Cho phép di chuyển ra xa ngay lập tức khi mục tiêu kéo ra ngoài. Cảnh báo chữ vàng hiển thị trên HUD: `Collision armor: BLOCKED <linkA> / <linkB> (X mm)`.
- **Mục tiêu ngoài tầm với (Out-of-Reach Target Projection & Recall/Tether):**
  - **Hình học tầm với:** $R_{\max} = 0.500$ m tính từ gốc khớp vai `left/right_arm_link01` (chiều dài chuỗi động học URDF là 0.510 m, trừ biên an toàn kỳ dị 10 mm).
  - **Chuột kéo Gizmo:** Trong lúc giữ chuột kéo ra xa, Gizmo di chuyển tự do theo con trỏ (client sở hữu hoàn toàn), hiển thị điểm bóng mờ (ghost sphere màu vàng) và tia nối đứt nét (dashed tether line) tại điểm chiếu $R_{\max}$. HUD hiển thị chữ vàng: `Target out of reach (X cm beyond)`. Khi nhả chuột nếu vượt quá tầm với $> 1$ cm, server gửi gói tin `gizmo_recall {arm, pos, duration_ms: 250}`. Client tự động chạy hiệu ứng lướt mượt mà (ease-out cubic trong 250 ms) đưa Gizmo về điểm chiếu chạm biên. Nếu người dùng bấm chuột kéo tiếp trong lúc đang lướt, hiệu ứng hủy tức thì tại vị trí hiện tại không nhảy giật. Khi nhả bên trong tầm với: không có tin nhắn recall, không đổi vị trí.
  - **Tay cầm VR & Hand Tracking (Tethering):** Khi bóp cò/grip kéo ra ngoài tầm với, mục tiêu lệnh được giữ trên mặt cầu giới hạn và anchor marker được dịch chuyển tương ứng (tether anchor shifting), khống chế tối đa `max_anchor_shift_m: 0.30` m (cấu hình trong `config/safety.yaml`). Khi tay lùi vào trong, cánh tay lập tức di chuyển theo ngay (vùng chết dead zone = 0 cm). Khi nhả ly hợp, xóa anchor và chốt giữ mục tiêu chiếu tại chỗ (`freeze_arm_target`), khi bóp lại neo theo vị trí hiện tại không nhảy giật.
  - **Cơ chế nới lỏng góc quay (Orientation Relaxation):** Khi vươn căng sát biên, bộ giải IK tự động ưu tiên thỏa mãn vị trí trước hướng quay (`position_cost=40.0, orientation_cost=0.5`); nếu không hội tụ, giữ nguyên tư thế hợp lệ cuối cùng (bit-identical). Tuyệt đối không dùng phản hồi đo thực để điều khiển hở vòng (open-loop command path).

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
   - Khi nhả Grip/Trigger: cánh tay robot bám tới vị trí đích cuối cùng, hội tụ êm và khóa đứng yên hoàn toàn (`TARGET_FROZEN`, không bị giật, không hunting).

### Cách B: Điều khiển bằng Chuột / 3D Gizmo trên trình duyệt Laptop
1. Mở trình duyệt trên máy trạm: `https://localhost:8012`
2. Kiểm tra thanh trạng thái hiển thị `READY (NOT ENGAGED)`.
3. Bấm nút **Engage** (hoặc nút **Home** / **T-Pose** / kéo Gizmo $\pm X, \pm Y, \pm Z$).
4. Robot sẽ chuyển sang trạng thái `ENGAGED` và bám theo mục tiêu 1:1 mượt mà.
5. Khi nhả chuột (`pointerup`, `pointercancel`, `blur`, `Escape`): Gizmo dừng đúng vị trí nhả chuột, robot bám tới đích và đứng yên tuyệt đối (`TARGET_FROZEN`).
6. Thanh trượt bàn tay (Hand Close) có tác dụng ngay từ đầu trên cả Sim và Robot thật.
7. Nếu di chuyển 2 tay lại quá gần ($< 50$ mm): HUD hiển thị cảnh báo chữ vàng `⚠️ Arm proximity: XX mm`, robot tự động giảm tốc mềm (không giật dừng, không khóa FAULT) và cho phép di chuyển lùi ra ngoài hoàn toàn tự do.
8. Nếu gặp chướng ngại hoặc chạm biên vật lý gây FAULT: bấm nút **Clear Fault** trên banner để đưa cánh tay về trạng thái an toàn `READY`.

---

## BƯỚC 5: DỪNG PHIÊN ĐIỀU KHIỂN AN TOÀN

- Khi kết thúc điều khiển: Bấm `Ctrl + C` trên Terminal SSH của Orin. Node sẽ tự động giữ nguyên vị trí đo thực trong 2 giây trước khi nhả lệnh.
- Nếu có bất thường: **Dập ngay nút E-Stop vật lý trên tay người giám sát** hoặc bấm tổ hợp nút `B + Y` trên tay cầm Quest.

---

## PHỤ LỤC: BẢNG BISECT DAO ĐỘNG & BIẾN MÔI TRƯỜNG (BISECT MATRIX)

Hệ thống hỗ trợ 6 cờ biến môi trường (mặc định giữ nguyên chuẩn, in rõ lúc khởi động trên cả Laptop và Orin):
- `CART_FOLLOWER=0/1` (Mặc định `1`): Bật/tắt bộ lọc Cartesian critically-damped trước IK (bộ lọc tiền IK duy nhất).
- `WRIST_SCALE=0.5/1.0` (Mặc định `0.5`): Hệ số giảm gia tốc cho 4 khớp cổ tay (Left: J5, J6; Right: J12, J13).
- `WIRE_HZ=50/100` (Mặc định `100`): Tần số phát UDP qua mạng.
- `PLAYOUT_MS=0/30` (Mặc định `30`): Độ trễ đệm khử jitter/drop gói tin trên Orin.

### Kiến trúc Limiter Đơn điệu Không Overshoot (Monotone No-Overshoot Architecture):
- **Thuật toán**: $v_{\text{des}} = \text{clamp}((target - x)/\tau, -v_{\max}, +v_{\max})$, $|dv/dt| \le a_{\max}$, $x \mathrel{+}= v \cdot dt$.
- **Hệ số $\tau$**: $\tau = \max(0.15, 1.2 \cdot v_{\max} / (2 a_{\max}))$, triệt tiêu hoàn toàn overshoot theo thiết kế hình học.
- **Cổ tay (J5, J6, J12, J13)**: $a_{\max} = a_{\max} \cdot \text{WRIST\_SCALE}$, sở hữu $\tau$ riêng tương ứng.
- **Loại bỏ**: Phanh parabol khoảng cách, `END_APPROACH`, và bộ lọc trùng lặp FIR (chỉ giữ Cartesian follower trước IK).
- **Đồng nhất Sim & Real**: Cả chế độ `--sim` và `real-udp` đều chạy chung một đường ống Limiter duy nhất.

### Cơ Chế Dừng Đột Ngột / Thả Chuột (Freeze-On-Release Architecture):
- **Trình duyệt (UI 8012)**: Gizmo thuộc quyền sở hữu của client trong và sau khi kéo (server không đẩy pose đè lên gizmo trừ nút Home/T-pose/Sync). Bắt trọn gói các sự kiện nhả/mất thao tác (`pointerup`, `pointercancel`, `lostpointercapture`, `mouseleave`, `blur`, `Escape`), lập tức gửi `gizmo_end` với pose mục tiêu cuối cùng và chặn toàn bộ các gói tọa độ phát sinh sau đó cho đến khi có thao tác kéo mới.
- **Laptop (Server)**: Khi nhận `gizmo_end` (hoặc timeout không có gói tọa độ sau 150 ms), hệ thống đóng băng (freeze) tọa độ Descartes, giải IK đúng 1 lần với độ chính xác cao ra $q_{\text{final}}$, sau đó ngắt cập nhật Cartesian follower và giải IK định kỳ.
- **Mirror Limiter**: Nhận mục tiêu khớp tĩnh $q_{\text{final}}$, hội tụ đơn điệu dưới gia tốc cho phép $a_{\max}$ và kích hoạt chốt tĩnh rest deadband/hold ($|e| \le 0.002$ rad, std = 0 qua 3s).
- **Tay cầm Quest VR**: Khi nhả Grip/Trigger chuyển về trạng thái `LOCKED`, tự động kích hoạt `freeze_arm_target` chốt cứng mục tiêu ngay lập tức.
- **Sự kiện Trace**: Laptop tự động ghi các sự kiện `GIZMO_START`, `GIZMO_END`, `TARGET_FROZEN`, `ARRIVED` (kèm `seq`) vào file `trace_laptop_<ts>.jsonl`.

### Cơ Chế Tự Va Chạm Giáp Bọc (Armor Collision Check & Soft Block):
- **Nguyên lý Giáp Bọc (Armor Shell)**: Mọi liên kết va chạm (collision links) trong URDF được thổi phồng thêm 1 cm (`shell_m: 0.010` m trong `config/safety.yaml`). Va chạm được tuyên bố khi hai lớp giáp chạm nhau, tương ứng khoảng cách bề mặt thực $d \le 2 \times \text{shell\_m} = 0.020$ m (20 mm).
- **Phạm vi kiểm tra**: 261 cặp va chạm ứng viên bao gồm tay-tay, tay-thân (`base_link`), tay-đầu (`head_link02`), tay-chân, và các khớp không liền kề trong cùng một cánh tay (loại trừ các cặp khớp liền kề và bàn tay chi tiết).
- **Bộ lọc bước quét (Swept Check & Bisection)**: Tại mỗi chu kỳ 100 Hz, limiter đánh giá 3 tư thế nội suy trung gian ($1/3, 2/3, 1$) của bước di chuyển kế tiếp. Nếu phát hiện vi phạm ngưỡng 20 mm, thuật toán chia đôi nhị phân (bisection: $1, 0.5, 0.25, 0.125$) để tìm bước di chuyển an toàn tối đa.
- **Khóa mềm (Soft Block)**: Khi bị chặn, cánh tay giữ nguyên tư thế hợp lệ cuối cùng với $v=0$ (PLAN bit-identical, std = 0, zero sign changes). Không phát sinh FAULT, không cần Clear Fault, không hiện banner đỏ.
- **Thoát tiếp xúc (Contact Retreat)**: Bất kỳ bước di chuyển nào làm tăng khoảng cách ($d_{\text{cand}} > d_{\text{curr}}$) đều được chấp thuận ngay lập tức, cho phép cánh tay rút ra khỏi vùng tiếp xúc mượt mà mà không bị kẹt.
- **Chốt trễ (Hysteresis)**: Trạng thái cảnh báo BLOCKED chỉ được nhả khi độ hở tăng thêm $\ge 2$ mm so với điểm tiếp xúc (hoặc vượt ngưỡng 20 mm).
- **Cảnh báo HUD**: Hiển thị dòng chữ vàng nổi bật trên giao diện: `Collision armor: BLOCKED <linkA> / <linkB> (X mm)`.
- **Hiệu năng & Đồng nhất**: Tối ưu hóa bằng bộ lọc hình cầu thô vector hóa (vectorized coarse bounding sphere pre-filter), thời gian tính toán $\approx 0.67$ ms/tick (dưới ngân sách 1.0 ms ở 100 Hz). Sử dụng chung 1 đối tượng/đường ống kiểm tra cho cả `--sim` và `real-udp`.

### Cơ Chế Xử Lý Mục Tiêu Ngoài Tầm Với & Độ Chính Xác IK (Reach Projection & IK Accuracy):
- **Phép chiếu Cartesian (Cartesian Reach Projection)**: Khi mục tiêu vượt ngoài tầm vươn ($R_{\max} = 0.500$ m tính từ khớp vai URDF `SHOULDER_ORIGIN_LEFT`: `[-0.03779, 0.19100, 1.27210]`, `SHOULDER_ORIGIN_RIGHT`: `[-0.03779, -0.19100, 1.27210]`), hệ thống tự động chiếu mục tiêu về mặt cầu biên $R_{\max}$ dọc theo hướng kéo.
- **Bảo toàn 1:1 trong tầm với**: Khi khoảng cách từ vai $d \le R_{\max}$, phép chiếu trả về chính xác 100% tọa độ mục tiêu (sai lệch $0.0$ mm).
- **Chỉ số sai số IK trên HUD**: HUD đo sai số giữa EE thực tế và mục tiêu *sau khi chiếu vào tầm với* (sai số solver thực sự). Phần vượt tầm với hiển thị riêng bằng cảnh báo vàng: `Target out of reach (X cm beyond)`.
- **Ưu tiên vị trí & Nới lỏng hướng (Position-First Relaxation)**: Trọng số `posture_task` được chuẩn hóa về $0.01$ (nullspace) giúp solver không bị giằng co với mục tiêu Cartesian. Ở biên tầm với hoặc góc khớp hẹp, trọng số hướng tự động nới lỏng ($3.0 \to 0.9 \to 0.15 \to 0.0$) để bảo đảm độ chính xác vị trí luôn $<3$ mm (HUD báo `(ori X°)` khi nới lỏng).
- **Chủ quyền Gizmo & Chốt thả**: Thao tác thả chuột/bóp nhả Grip chốt cứng mục tiêu và hội tụ IK mượt mà; loại bỏ hoàn toàn lỗi kẹt 277 mm.

### Thanh Trượt Điều Khiển Bàn Tay (Hand Slider):
- **Hiệu lực tức thì**: Thanh trượt bàn tay hoạt động ngay lập tức từ khi vừa bật máy (`fresh start`) ở cả `--sim` và `real-udp` mà không cần bấm Open/Close trước.
- **Thứ tự thao tác**: Có thể tự do xen kẽ thanh trượt và các nút Open / Close / Pinch / Home / T-pose theo bất kỳ thứ tự nào.
- **Giới hạn an toàn**: Lệnh bàn tay được mã hóa trong khoảng $[0..800]$ raw (tối đa 800) với dòng lực an toàn default 1140 (20%), được kiểm tra va chạm ngón tay trước khi xuất lệnh.

### Trạng Thái Orin rx_state & Khởi Động Lại:
- `RX_OK`: Nhận lệnh bình thường và đang xuất điều khiển robot.
- `NOT_ENGAGED`: Chưa bấm Engage trên Laptop / chưa bóp Grip.
- `HOLDING`: Session lock bị giữ bởi phiên cũ hoặc IP lạ. Khắc phục: Dừng gửi 2s để Orin tự nhả lock, sau đó Engage lại.
- `REJECT:<reason>`: Gói tin bị từ chối do quá cũ (>0.4s) hoặc sai định dạng.
- `FAULT:<reason>`: Bị chốt lỗi an toàn (ví dụ phát hiện node khác gửi đè). Cần restart `arm_teleop_rx.py`.

---

## QUY TRÌNH THU THẬP TRACE & TỰ ĐỘNG CHẨN ĐOÁN
1. **Terminal 1 (Orin - duy nhất 1 lệnh chạy receiver):**
   ```bash
   cd /agibot/data/home/agi/Desktop/mc
   python3 arm_teleop_rx.py
   # Tự động xuất logs/trace_orin_<ts>.jsonl và logs/trace_recorder_<ts>.jsonl
   ```
2. **Terminal 2 (Laptop - Teleop Server):**
   ```bash
   cd ~/a2_ws && source venv/bin/activate
   python3 main.py
   # Tự động xuất logs/trace_laptop_<ts>.jsonl khi truyền UDP
   ```
3. **Phân tích kết quả sau khi kéo di chuyển và dừng (Ctrl+C cả 2 terminal):**
   ```bash
   # Kéo 2 file log từ Orin về:
   scp "agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/trace_*.jsonl" logs/
   # Chạy phân tích:
   python3 scripts/classify_arrival.py logs/trace_laptop_<ts>.jsonl logs/trace_orin_<ts>.jsonl logs/trace_recorder_<ts>.jsonl
   ```
   **Kỳ vọng chẩn đoán sau khi fix:**
   - `SENT ptp < 0.003 rad` (hết hunting hoàn toàn trên Laptop).
   - Phán quyết: `V4 (ROBOT_INNER_LOOP)` nếu khớp còn rung cơ học tự nhiên hoặc kết quả dừng ổn định hoàn hảo.
