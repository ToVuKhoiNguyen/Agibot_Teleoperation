# Khắc Phục Sự Cố, Điểm Chưa Xác Định & Vệ Sinh Phát Hành

---

## 1. Ma Trận Khắc Phục Sự Cố Thường Gặp (Troubleshooting Matrix)

| Hiện tượng | Nguyên nhân gốc | Hướng xử lý |
| :--- | :--- | :--- |
| **Không nhận được chủ đề ROS 2** | Robot chưa ở chế độ Servo hoặc sai dải IP | Ping `192.168.100.100` từ Orin (laptop không có đường tới máy X86 192.168.100.100, nên không gửi lệnh tay qua ROS 2 trực tiếp từ laptop được); chạy lệnh chuyển chế độ `tools/aima_mode.py` |
| **Sai lệch Domain DDS** | Biến `ROS_DOMAIN_ID` không khớp máy thật | Thiết lập đúng `export ROS_DOMAIN_ID=232` (hoặc ID do hãng cung cấp) |
| **FastDDS không tìm thấy nhau** | Gói tin multicast bị chặn giữa Jazzy và Humble | Thiết lập biến môi trường `export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` |
| **Mất tín hiệu điều khiển (Watchdog)** | Kính Quest mất tầm nhìn hoặc tay cầm bị che | Ly hợp tự động khóa sau $350\text{ ms}$; đưa tay về góc nhìn để bắt lại |
| **Kích hoạt chốt lỗi FAULT** | Dữ liệu trễ $> 200\text{ ms}$ hoặc lệch góc $> 0.05\text{ rad}$ | Giữ nguyên robot; bấm "Clear Fault" trên giao diện Web để mở lại |
| **Không có nút ENTER VR trên Quest** | Kết nối qua HTTP thường hoặc trình duyệt lạ | Truy cập bằng HTTPS: `https://localhost:8012/` trên Meta Quest Browser |
| **Lỗi cổng 8012 đã bị chiếm dụng** | Một tiến trình python cũ vẫn đang chạy ngầm | Chạy `fuser -k 8012/tcp` hoặc kiểm tra PID qua `ss -tulpn \| grep 8012` |
| **Lỗi import rclpy trong venv** | Môi trường ảo Python bị cô lập khỏi ROS 2 | Khởi tạo lại venv với cờ `--system-site-packages` |

---

## 1a. "JOINT NOT FOLLOWING" / tay đi lạ, phải xoay tay mới đi đúng, bàn tay chạm chân (17/11)

Một khớp không còn được MC điều khiển: lệch lệnh > 0.35 rad quá 1 s; laptop báo tên khớp, lệnh và giá trị đo. Ngày 17/11 là khuỷu (J4), sau cú đập do lỗi pacer 12:00–14:00 (đã sửa, xem `real_robot_command.md` mục SỰ CỐ KHUỶU 17/11).

Xử lý:
1. Dừng teleop tay đó.
2. Tắt nguồn robot, bật lại, SetAction 10 → 12.
3. Kiểm tra: `python3 joint_limit_sweep.py --joints 4 --speeds 0.4 --strokes 2`.
4. Nếu vẫn lỗi: dùng AimMaster hoặc báo AgiBot.

Kiểm tra trong log laptop: `logs/real_<giờ>.jsonl` có `target_q` (lệnh) và `measured_q` (đo).

## 1b. Tay Robot Rung Hoặc Giật (cập nhật 17/11)

> Phân tích nguyên nhân cốt lõi, số đo bộ lọc và cách xác nhận bằng `cmd_recorder.py` + `vibration_report.py`: [FILTER_REPORT.md](FILTER_REPORT.md).

Làm theo thứ tự; mỗi bước chỉ đổi một thứ.

1. **Kiểm tra receiver trên Orin**: dòng khởi động phải ghi `[pacer] arm command pacer running: 400 Hz` và `lvl=` trùng `--speed` của `main.py` (mặc định 6). Ghi 100 hay 200 Hz là bản cũ: chép lại `scripts/arm_teleop_rx.py` và `scripts/orin_teleop_core.py` (mã băm trong `real_robot_command.md`, BƯỚC 0).
2. **Rung mịn, tỉ lệ với tốc độ** (đo 13/11: 50 Hz ~14%, 100 Hz ~10% tốc độ, 200 Hz đỡ rõ, 400 Hz ít nhất): là phản ứng của MC với lệnh vị trí dạng bậc, rõ nhất ở cổ tay J6/J7. Đo 17/11: MC 402 tự dao động ~10 Hz tỉ lệ với tốc độ khớp kể cả với lệnh dốc hoàn hảo phát ngay trên Orin (gấp ~4 lần PlanningMove), nên phần rung này không do mạng hay bộ lọc; cổ tay J6/J7 rung gấp ~20 lần lệnh mượt của chính nó, tức bị vai / khuỷu truyền xuống. Giảm bằng cách giữ 400 Hz và hạ tốc độ khớp: `--speed 5`, `4` hoặc `3` ở CẢ HAI bên, hoặc chỉ chặn vai / khuỷu trên Orin bằng cấu hình D (`JOINT_VMAX=1.2,1.5,1.2,1.5,2,2,2 JOINT_AMAX=6 python arm_teleop_rx.py`, D1 với `JOINT_AMAX=8`; laptop giữ `python main.py`). Hạ tốc độ cổ tay không giúp. Mode 205 không có trong menu SetAction của robot này; PlanningMove (403) thử cho Home 17/11 chậm và giật nên đã bỏ.
2a. **Bàn tay rung trên đường đi khi dùng tay cầm**: mặc định khi chỉ giữ trigger, hai khớp cổ tay giữ nguyên góc (`XR_SOFT_ORI`) và lệnh ngón tay được giữ nguyên khi tay đang chạy (`HAND_HOLD_MOVING`). So sánh bằng `XR_SOFT_ORI=0 HAND_HOLD_MOVING=0 python main.py`. Giao diện của hãng không có lệnh khóa hay chỉnh độ cứng cổ tay; nếu lắc tay robot bằng tay thấy rơ ở cổ tay thì là việc cơ khí.
2b. **Bàn tay lắc quanh cổ tay lúc dừng (rõ nhất ở tư thế HOME)**: cảm biến khớp không thấy dao động, tức là phần cơ khí sau cổ tay bị kích bởi cú phanh. Mặc định (17/11, `--smooth 1`) lọc đầu ra laptop 4 tick, không có lọc thứ hai; `--smooth 3` = 6 + 4 tick (`DIRECT_OUT_FIR=6 DIRECT_OUT_FIR2=4`); nếu lắc nhiều hơn, quay lại bản mượt cũ `DIRECT_OUT_FIR=12 DIRECT_OUT_FIR2=10 python main.py` (trễ thêm ~60 ms), `DIRECT_OUT_FIR2=1` để tắt và so sánh.
3. **Giật từng nấc khi bấm nút dịch XYZ, hoặc giật lúc nhả / bóp trigger**: do phía laptop. Tắt hết các thay đổi để so sánh:
   `DIRECT_OUT_FIR2=1 XR_SOFT_ORI=0 HAND_HOLD_MOVING=0 SOFT_ORI=0 IK_SETTLE_S=0 XR_COAST=0 NUDGE_GLIDE=0 python main.py --elbow-out 0`
   Nếu đỡ, bật lại từng công tắc (bỏ dần từng biến) để tìm đúng cái gây ra.
4. **Vẫn giật với bước 3**: kiểm tra dòng `[pacer]` có `SCHED_FIFO 30 (real-time)`; nếu không, chạy `sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40` trong terminal đó rồi chạy lại receiver (Orin bỏ đói pacer = lệnh cách nhau tới 140 ms, 16/11). Vẫn giật thì thử `PACER_CPU=11` hoặc `PACER_HZ=200`.
5. **Đo khách quan** (trên Orin, không cần laptop, receiver phải tắt): `python arm_stream_test.py --compare` in bảng độ rung theo % tốc độ cho 100 / 200 / 400 Hz và phát khóa theo nhịp MC.
5a. **Receiver báo FAULT `pacer process stopped (exit code N, ...)` hoặc `pacer process frozen > 0.5 s`**: xem các dòng `[pacer]` ngay trên trong terminal Orin. Pacer luôn in `[pacer] stopped: <lý do>` (kèm traceback nếu crash). Receiver khựng > 0.5 s thì pacer chỉ GIỮ tay (`[pacer] parent silent ... -> HOLD arm`, rồi `resumed`), chỉ thoát khi im quá 3 s hoặc receiver chết. Đọc log của `arm_stream_test`: `python tools/stream_test_report.py <log>`.
6. **Gửi log để phân tích**: `scp 'agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/trace_orin_*.jsonl' logs/` (log phía laptop không đo được độ rung thật của robot).

## 2. Danh Sách Các Điểm Chưa Xác Định Từ Nhà Sản Xuất (Known Unknowns)
1. **Thời gian ngắt của Hardware Watchdog trên robot**: Tài liệu kỹ thuật của hãng không công bố rõ thời gian ngắt; laptop gửi UDP 100 Hz, pacer trên Orin phát lệnh 400 Hz; receiver báo lỗi `stale feedback` nếu phản hồi trễ quá $300\text{ ms}$.
2. **Cơ chế ngắt mạch của nút E-Stop phần cứng**: Cần trang bị nút dừng khẩn cấp không dây riêng; giả định động cơ mất nguồn và cánh tay thả lỏng $\to$ bắt buộc phải dùng khung gantry đỡ trọng lượng.
3. **Chiều dương của khớp xoay ngón cái (`thumb_0`)**: Giao thức AimDK không chỉ rõ; từ 17/11 ngón cái mặc định nghỉ ở tư thế cầm vật (400/900, tối 17/11 hạ từ 800 để bớt lắc) và tới xoay 800 / gập 1500 khi nắm (`--thumb-pose 400,900` = xoay 400, gập 900 raw; lực giữ ngón cái `THUMB_EFFORT=2280`; `--thumb-pose off` = theo tay như cũ, nghỉ ở `--thumb-rest` 500); nếu thấy ngón cái xoay ngược chiều, chạy receiver với `--no-thumb-swing` để giữ `thumb_0` ở 0.
4. **Động học tay đẩy thanh giằng cổ tay**: Mô hình động học nghịch (IK) giả định tương đương với cơ cấu nối tiếp chuẩn.
5. **Giới hạn tải trọng**: Tải trọng danh định 1 kg một tay / 3 kg lực kẹp; phạm vi với tối đa chưa được nhà sản xuất chuẩn hóa.
6. **Bản quyền file lưới 3D**: Toàn bộ file lưới STL/OBJ trong thư mục `assets/` được trích xuất phục vụ mục đích nghiên cứu học thuật nội bộ, chưa được xác nhận giấy phép thương mại từ hãng.
7. **Trạng thái trên phần cứng vật lý**: Hệ thống đã chạy teleop trên robot thật (từ 12/11); nhiều thay đổi mới nhất (đánh dấu trong `real_robot_command.md`) mới kiểm bằng mô phỏng / SIL.

---

## 3. Quy Tắc Vệ Sinh Phát Hành & Quy Trình Git (Hygiene per AGENTS.md)
- **Tuyệt đối không push tùy tiện**: Chỉ thực hiện `git push` khi người dùng yêu cầu rõ ràng.
- **Không bao giờ dùng `--force` hoặc viết lại lịch sử remote**.
- **Kiểm soát dung lượng và bí mật**:
  - Không commit bất kỳ file nào có dung lượng $> 5\text{ MB}$.
  - Không commit file khóa, chứng chỉ (`*.key`, `*.pem`), biến môi trường (`.env`), thư mục tạm `venv/`, `_trash/`, `logs/`.
- **Kiểm tra độ dài tệp tin nguồn**: Toàn bộ các file mã nguồn và tài liệu phải tuân thủ giới hạn $\le 250$ dòng (`python tools/check_file_sizes.py`).
- **Quy chuẩn thông điệp Commit**: Sử dụng cú pháp Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, `refactor:`).
