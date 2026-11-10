# Hướng Dẫn Vận Hành Bộ Thu Teleop Trên Orin (`arm_teleop_rx.py`)

> **CẢNH BÁO QUAN TRỌNG:**
> Mọi hành vi điều khiển phần cứng thực tế đều là **CHƯA ĐƯỢC KIỂM CHỨNG TRÊN PHẦN CỨNG THẬT (UNVERIFIED ON REAL HARDWARE)**.
> Tài liệu này và mã nguồn kèm theo được phát triển trên môi trường giả lập/loopback laptop (`ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`).
> **TUYỆT ĐỐI KHÔNG** kết nối robot, không chạy lệnh khi robot chưa được treo cáp an toàn (gantry/crane), và luôn sẵn sàng nút dừng khẩn cấp cơ học (E-Stop).

---

## 1. Mối Nguy Hiểm & Phân Tích Mã Nguồn Mẫu Của Vendor (`arm.py`)

Tệp mã nguồn mẫu `arm.py` của vendor (`examples/mc/arm.py`) tồn tại 5 mối nguy hiểm vật lý nghiêm trọng:

1. **Khởi tạo tức thời không bám trạng thái thực (Tick-1 Instant Snap):**
   * *Hiện tượng:* Gửi ngay lập tức mảng góc cứng `pos_ini = [0, 1.2, 0, -0.5, 1.5, 0, 0, 0, -1.2, 0, 0.5, 1.5, 0, 0]` ngay chu kỳ đầu tiên mà không đọc góc khớp thực tế hiện tại.
   * *Hậu quả:* Nếu hai tay robot đang buông thõng hoặc ở tư thế gập, khớp sẽ giật tức thời với vận tốc cực đại, gây va đập cơ khí gãy khớp hoặc kích hoạt rơ-le ngắt nguồn khẩn cấp.
2. **Tự ý chuyển trạng thái máy trạng thái (Forced Action Switch):**
   * *Hiện tượng:* Tự động gọi `ensure_action("McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO")` can thiệp vào bộ điều khiển chuyển động của robot qua RPC.
   * *Hậu quả:* Cướp quyền điều khiển khi người vận hành chưa sẵn sàng hoặc robot chưa vào trạng thái an toàn.
3. **Không có bộ lọc và giới hạn gia tốc/vận tốc (Missing Safety Limits):**
   * *Hiện tượng:* Không kiểm tra giới hạn khớp vật lý (intersection limits), không kẹp vận tốc tối đa và không có bộ lọc làm mịn S-curve/low-pass.
   * *Hậu quả:* Bất kỳ bước nhảy mục tiêu nào từ mạng cũng gây vọt lố (overshoot) và phá hủy hộp số hài.
4. **Xung đột bộ phát song song (Second Publisher Conflict):**
   * *Hiện tượng:* Nếu tiến trình `motion_player` của vendor đang chạy ngầm, nó cũng phát vào `/motion/control/arm_joint_command`.
   * *Hậu quả:* Hai bộ phát đè dữ liệu xen kẽ ở tần số cao gây rung giật dữ dội (jitter) làm cháy động cơ servo.
5. **Thiếu cơ chế giám sát mất tín hiệu (No Watchdog / Timeout Hold):**
   * *Hiện tượng:* Vòng lặp đơn giản `while True: sleep(0.01)` không có bộ đệm kiểm tra ngắt kết nối.
   * *Hậu quả:* Khi mạng rớt hoặc laptop bị treo, robot giữ lệnh cuối cùng hoặc hành vi không xác định.

---

## 2. Thiết Kế An Toàn Của `arm_teleop_rx.py`

Node thu an toàn được thiết kế với các nguyên tắc phòng vệ nghiêm ngặt:
- **Kiểm tra Action ở chế độ chỉ đọc (Read-only Check):** Không bao giờ gọi `set_action` hay `ensure_action`. Người vận hành phải tự chuyển action thủ công qua giao diện vendor. Node chỉ phát lệnh khi action đã ở đúng `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`.
- **Mặc định `--dry` an toàn tuyệt đối:** Mặc định chạy ở chế độ dry-run, chỉ lắng nghe UDP và đọc trạng thái `/motion/control/arm_joint_state`, phát đúng 0 message ra bus điều khiển.
- **Cổng an toàn 5 bước để kích hoạt `--live`:**
  1. Cờ dòng lệnh bắt buộc `--live`.
  2. Cờ xác nhận rủi ro vật lý `--i-understand-hardware-risk`.
  3. Chỉ định IP laptop cụ thể `--allow-ip <IP>` (từ chối tất cả các IP lạ).
  5. Đã hoàn thành bước kiểm tra `--dry` trước đó không có lỗi.
- **Neo trạng thái thực (Measured State Anchoring):** Lệnh đầu tiên phát ra bus luôn bằng chính xác góc đo thực tế hiện tại ($\Delta q < 10^{-6}\text{ rad}$). Tuyệt đối không giật vị trí khi bắt đầu.
- **Giới hạn kép (Intersection Limits & Dynamic Rate Limiting):**
  - Kẹp góc theo vùng giao an toàn chặt chẽ nhất của bảng giới hạn 7.1.9 và vùng vàng vendor.
  - Kẹp vận tốc mặc định $0.3\text{ rad/s}$ (tối đa $1.0\text{ rad/s}$ nếu có cờ `--i-accept-higher-speed`).
  - Kẹp gia tốc tối đa $2.0\text{ rad/s}^2$ tích hợp theo thời gian thực $\Delta t$.
- **Giữ vị trí an toàn khi mất gói (Watchdog Hold):** Nếu khoảng cách giữa 2 gói UDP $> 300\text{ ms}$, giữ nguyên vị trí hiện tại.
- **Chốt lỗi lệch góc (Deviation Latch Gate):** Nếu độ lệch giữa lệnh phát và góc đo thực tế $> 0.05\text{ rad}$ kéo dài quá $200\text{ ms}$, chốt lỗi FAULT và khóa cứng vị trí tại góc đo thực tế.
- **Giữ vị trí khi thoát (Safe Shutdown Hold):** Khi nhận `Ctrl-C` (SIGINT), tiếp tục phát giữ góc đo thực tế trong 2.0 giây để triệt tiêu toàn bộ quán tính trước khi giải phóng node.

---

## 3. Giao Thức Truyền Dẫn Mạng UDP JSON (Laptop $\to$ Orin)

### 3.1. Lý do không dùng DDS trực tiếp từ Laptop qua Wi-Fi/Ethernet
- Discovery của CycloneDDS/FastDDS qua mạng không dây thường xuyên bị rớt multicast, nghẽn hàng đợi gói và gây jitter chu kỳ điều khiển rất lớn.
- Khác biệt cấu hình ROS 2 (Humble trên Ubuntu 22.04 Orin vs Jazzy trên laptop) dễ gây mất tương thích serialization gói tin.
- **Giải pháp:** Sử dụng UDP JSON unicast trực tiếp từ Laptop đến Orin port 5005. Trên Orin, `arm_teleop_rx.py` chuyển tiếp sang DDS nội bộ qua IPC/shm ở tần số ổn định 100 Hz.

### 3.2. Cấu trúc gói tin UDP JSON
```json
{
  "seq": 1042,
  "t": 1793951833.120,
  "q": [0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
}
```
- `seq`: Số thứ tự tăng đơn điệu. Bộ thu loại bỏ ngay các gói đến sau nhưng có seq nhỏ hơn (out-of-order).
- `t`: Thời gian phát epoch giây. Loại bỏ gói nếu $|t_{\text{now}} - t| > 0.200\text{ s}$ (stale).
- `q`: Mảng đúng 14 giá trị float tương ứng 14 khớp `idx13`..`idx26`. Loại bỏ nếu chứa NaN/Inf hoặc độ dài khác 14.

### 3.3. Cổng dự phòng bảo mật SSH Tunnel
Nếu mạng nội bộ chặn UDP hoặc cần bảo mật mã hóa đường truyền:
```bash
# Thiết lập SSH Tunnel từ Laptop đến Orin cổng 5005
ssh -N -L 5005:127.0.0.1:5005 agi@<ORIN_IP>
```
Lúc này Laptop gửi UDP đến `127.0.0.1:5005`, Orin cấu hình `--allow-ip 127.0.0.1`.

---

## 4. Quy Trình Kiểm Thử 6 Giai Đoạn (T0 $\to$ T5)

### T0: Đo Kiểm Benchmark Loopback Trên Laptop (Đã xác minh 100% PASS)
- Môi trường: Laptop cục bộ, `ROS_DOMAIN_ID=77`, `ROS_LOCALHOST_ONLY=1`.
- **Kết quả đo kiểm thực tế:**
  1. `--dry` phát đúng **0 commands** ra bus điều khiển.
  2. Lệnh đầu tiên trong `--live` khớp góc đo thực: sai số lệch **$0.00000000\text{ rad}$**.
  3. Đáp ứng xung bước $0.3\text{ rad}$: vận tốc đỉnh đạt **$0.3019\text{ rad/s}$** (ngưỡng kẹp $\le 0.315\text{ rad/s}$); gia tốc đỉnh đạt **$2.1282\text{ rad/s}^2$** (ngưỡng kẹp $\le 2.25\text{ rad/s}^2$).
  4. Kẹp giới hạn biên: gửi mục tiêu vọt $5.0\text{ rad}$, bộ thu kẹp an toàn về **$0.3516\text{ rad} \le 2.91\text{ rad}$**.
  6. Tần số phát lệnh: **$100.0\text{ Hz}$** chu kỳ ổn định với độ trễ jitter phân vị **$p99 = 10.22\text{ ms} < 15\text{ ms}$**.
  7. Dừng an toàn khi nhận `SIGINT`: hoàn tất chu trình giữ vị trí và ngắt sạch.
  8. Kiểm tra tĩnh: 0 lời gọi tới `set_action`, `ensure_action`, hay `SetAction`.

### T1: Kiểm Tra Đường Cơ Sở Chỉ Đọc Trên Orin (Read-Only Baseline)
1. Kết nối Orin qua cáp mạng Ethernet vật lý trực tiếp.
2. Kiểm tra tiến trình xung đột `motion_player`:
   ```bash
   ps aux | grep motion_player
   # Nếu đang chạy, PHẢI dừng: killall -9 motion_player
   ```
3. Kiểm tra tần số và dữ liệu trạng thái khớp:
   ```bash
   ros2 topic hz /motion/control/arm_joint_state
   ros2 topic echo /motion/control/arm_joint_state --once
   ```
4. Kiểm tra quyền sở hữu topic lệnh:
   ```bash
   ros2 topic info /motion/control/arm_joint_command
   # Đảm bảo Publisher count = 0
   ```

### T2: Tải Mã Nguồn Lên Orin Có Cổng Kiểm Soát (Gated Upload)
1. Tạo thư mục làm việc riêng biệt trên Orin:
   ```bash
   mkdir -p ~/teleop_deploy && cd ~/teleop_deploy
   ```
2. Chép tệp `arm_teleop_rx.py` lên Orin qua `scp`.
3. Kiểm tra cú pháp Python 3.10 trên Orin:
   ```bash
   python3 -m py_compile arm_teleop_rx.py
   ```
4. Kiểm tra tĩnh để khẳng định mã không thể tự chuyển action:
   ```bash
   grep -nE "set_action|ensure_action|SetAction" arm_teleop_rx.py
   # Kết quả BẮT BUỘC phải là rỗng (0 dòng)
   ```

### T3: Kiểm Thử Chế Độ Bắt Gói Không Phát Lệnh (Dry Run)
1. Chạy trên Orin ở chế độ mặc định:
   ```bash
   python3 arm_teleop_rx.py --dry --allow-ip <LAPTOP_IP> --port 5005
   ```
2. Trên Laptop, gửi chuỗi UDP kiểm thử.
3. Trên Orin, mở terminal thứ hai kiểm tra:
   ```bash
   ros2 topic echo /motion/control/arm_joint_command
   # BẮT BUỘC: Không có bất kỳ message nào xuất hiện (count = 0).
   ```

### T4: Kích Hoạt Có Cổng Kiểm Soát Trên Robot Treo Cáp (Gated Live Run)
1. **ĐIỀU KIỆN TIÊN QUYẾT BẮT BUỘC:**
   - Robot đã được treo hoàn toàn trên cáp bảo hiểm (crane/gantry), hai chân không chạm sàn, vùng làm việc của hai tay cách xa mọi vật cản $\ge 1.5\text{ m}$.
   - Nút E-Stop vật lý nằm sẵn trên tay người quan sát.
   ```bash
   ```
3. Khởi chạy với đầy đủ 5 cổng an toàn:
   ```bash
   python3 arm_teleop_rx.py --live --i-understand-hardware-risk --allow-ip <LAPTOP_IP> --vel-limit 0.3 --accel-limit 2.0
   ```
4. Xác nhận log: `Anchored to measured pose` xuất hiện và hai tay robot đứng yên tuyệt đối, không có tiếng rít động cơ.

### T5: Nhấp Nhả Khớp Nhỏ Từng Bước (Incremental Micro-Jog)
1. Chỉ gửi dịch chuyển cực nhỏ: $\Delta q = 0.01\text{ rad}$ ($\sim 0.5^\circ$) trên từng khớp đơn lẻ (bắt đầu từ khớp cổ tay ngoài cùng).
2. Kiểm tra khớp di chuyển êm dịu, không giật cục, không phát sinh rung lắc.
3. Thử nghiệm ngắt mạng (rút cáp Ethernet): Robot lập tức dừng êm và giữ nguyên vị trí trong vòng $300\text{ ms}$, không bị sập hay giật.

---
*Ghi chú: Xem bảng quy tắc dừng khẩn cấp và danh mục kiểm tra trước khi vận hành tại `docs/ORIN_TELEOP_CHECKLIST.md`.*

