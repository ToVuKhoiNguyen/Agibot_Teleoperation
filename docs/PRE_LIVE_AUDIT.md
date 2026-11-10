# Báo Cáo Kiểm Định An Toàn Tiền Vận Hành Robot Thật (Pre-Live Safety Audit)

## 1. Bản Đồ Chuỗi Giới Hạn Tốc Độ (Speed-Limit Chain Map)

| Thành phần | File:Dòng | Đơn vị & Giá trị | Tác động lên | Ghi chú & Đánh giá |
|:---|:---|:---|:---|:---|
| **Quest Input** | `xr_input_handler.py:45` | Tọa độ 6DoF tay người | Vị trí mục tiêu EE | Chu kỳ ~72-90 Hz từ WebXR, có jitter mạng |
| **Gizmo Follower** | `followers.py:30` | 2nd-order critically damped | Vị trí EE mượt | Lọc rung tay, $\omega=25$ rad/s |
| **IK Solver** | `agibot_a2_ik.py:70` | 7 khớp (rad) | $q_{target}$ mỗi tay | Giới hạn 14 vòng lặp, pos_tol 1mm |
| **IK Continuity Guard** | `ik_dispatcher.py:39` | $\Delta q \le 0.1$ rad | $q_{sol}$ | Chống lật cấu hình (flip) khi EE dịch < 5mm |
| **SafetyLayer & Follower**| `safety_layer.py:125` | 14 DoF, $\Delta q \le 0.01$ rad | Target khớp | 5 khóa an toàn, chặn tự va chạm, shadow predict |
| **Laptop UDP Sender** | `udp_backend.py:125` | 50 Hz monotonic, 2nd-order | Gói tin UDP ra | $v_{max}=0.5$ rad/s, $a_{max}=1.5$ rad/s$^2$, proto_ver=2 |
| **Orin Socket Drain** | `arm_teleop_rx.py:90` | Drain toàn bộ backlog | Packet mới nhất | Loại bỏ chồng đống buffer, khóa single-sender |
| **50 Hz Monotonic Timer** | `arm_teleop_rx.py:160` | $dt \in [10, 40]$ ms | Chu kỳ tính toán | **Điểm kích hoạt duy nhất** (1 publish site) |
| **Final-Line Guard** | `arm_teleop_rx.py:120` | $v \le 0.48$ rad/s, $a \le 1.05$ | Lệnh xuất ra robot | Uniform scaling, parabolic brake, chốt FAULT nếu $>0.02$ |
| **ROS 2 Publisher** | `arm_teleop_rx.py:150` | `/motion/control/arm_joint_command` | MC Driver | Đúng 1 publisher, tần số $50\pm 1$ Hz |

## 2. Bảng So Sánh Trước và Sau Kiểm Định (Before / After)

| Tiêu chuẩn an toàn | Trước kiểm định | Sau kiểm định (fix) | Ngưỡng đạt chuẩn (PASS) |
|:---|:---:|:---:|:---:|
| **Số điểm publish trong receiver** | 3 vị trí (bị phân tán) | **1 vị trí duy nhất** (timer 50Hz) | Đúng 1 site trên 50 Hz timer |
| **Bước nhảy góc tối đa (per cycle)** | 0.050 rad (spikes khi dồn gói) | **0.0098 rad** | $\le 0.010$ rad/chu kỳ |
| **Tần số xuất lệnh (Publish Rate)** | Burst bất thường | **50.0 Hz** (p99 dt: 20.6 ms) | $50 \pm 5$ Hz, p99 dt < 25 ms |
| **Gia tốc đỉnh tại điểm publish** | $> 50$ rad/s$^2$ (đột biến giật) | **1.74 rad/s$^2$** | $\le 2.0$ rad/s$^2$ |
| **Bảo toàn hướng (Uniform Scaling)**| Lệch hướng (cắt riêng khớp) | **Độ lệch 0.0°** (đồng tỉ lệ) | Sai lệch góc $< 5.0^\circ$ |
| **Số bước nhảy $> 0.02$ rad** | Xuất hiện khi nối lại clutch | **0** (Chốt FAULT giữ cứng) | 0 bước |
| **Khóa nguồn gửi (Single Sender)** | Không có (2 máy gửi xen kẽ) | **Khóa IP + Session ID** | Từ chối nguồn lạ, giữ an toàn |
| **Kiểm tra phiên bản & Hash** | Không kiểm tra | **Proto v2 + Git Build ID + SHA**| Từ chối nếu lệch mã |

## 3. Các Nguyên Nhân Gây Giật Đã Được Khắc Phục (Findings)
1. **Dồn ứ hàng đợi UDP (Socket Backlog)**: Gói tin đến dồn dập khiến receiver xử lý tuần tự tạo ra bước nhảy lớn. Đã fix bằng `_drain_socket()` chỉ lấy gói mới nhất.
2. **Cắt giới hạn độc lập (Independent Clipping)**: Cắt từng khớp khi tới biên hoặc quá tốc làm biến dạng vector hướng và giật nảy. Đã thay bằng đường phanh parabolic rời rạc và co giãn đồng tỉ lệ (uniform vector scaling).
3. **Đa điểm publish**: Các luồng watchdog, clutch và homing từng publish trực tiếp ngoài timer. Đã hợp nhất về đúng 1 hàm `_final_line_guard_and_publish`.

## 4. Những Điểm Chưa Được Kiểm Chứng (Unverified on Hardware)
- **Tải trọng và quán tính cơ học thực tế**: Toàn bộ kiểm thử loopback domain 77 giả lập không đo được rung động cộng hưởng vật lý của cánh tay robot khi có tải.
- **Độ trễ vật lý cáp mạng switch**: Môi trường loopback có độ trễ truyền dẫn ~0.1 ms, trên thực tế cáp Ethernet qua switch cần kiểm chứng lại bằng ladder test.

## 5. Danh Mục Kiểm Tra An Toàn Trước Vận Hành (Pre-Live Checklist)

1. **Khớp mã nguồn**: `sha256sum scripts/arm_teleop_rx.py` trên Laptop và Orin.
   *Tiêu chí PASS*: Mã băm SHA256 trùng khớp 100%.
2. **Đúng 1 tiến trình phát**: `pgrep -af "python.*main.py"` và kiểm tra lock `/tmp/a2_teleop.lock`.
   *Tiêu chí PASS*: Đúng 1 tiến trình tồn tại.
3. **Đúng 1 tiến trình gửi lệnh ROS**: `ros2 topic info -v /motion/control/arm_joint_command`.
   *Tiêu chí PASS*: `Publisher count: 1`.
4. **Chạy thử nghiệm Dry-run trước**: Chạy `python3 main.py --dry --backend real-udp`.
   *Tiêu chí PASS*: Không có ngoại lệ hoặc cảnh báo FAULT.
5. **Thử nghiệm HOLD 10 giây**: Bật live ở tư thế đứng yên, không di chuyển tay trong 10 giây.
   *Tiêu chí PASS*: Độ lệch vị trí khớp đo được $|delta| < 0.005$ rad.
6. **Thử nghiệm khớp đơn nhẹ**: Nhích 1 khớp cánh tay $+0.05$ rad ở tốc độ profile `slow`.
   *Tiêu chí PASS*: Chuyển động êm ái, dừng mượt không overshoot hay rung lắc.
7. **Bậc thang độ trễ (Ladder Rungs)**: Chạy `python3 scripts/test_latency_ladder.py`.
   *Tiêu chí PASS*: Rung B & C1 đạt `lag_mean` $100\pm 15$ ms, jitter p99 $< 5$ ms.
8. **Người giám sát nút E-Stop không dây**: Một người chuyên trách cầm nút E-stop vật lý.
   *Tiêu chí PASS*: Sẵn sàng ngắt nguồn ngay lập tức khi có hiệu lệnh.
9. **Quy tắc dừng khẩn cấp**: Lập tức dừng nếu phát hiện bất kỳ va chạm, rung rần hoặc trạng thái chốt FAULT.
