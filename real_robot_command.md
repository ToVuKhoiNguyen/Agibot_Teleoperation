# Hướng Dẫn Vận Hành Teleop Robot Thật Bằng Kính Quest 3

Tài liệu hướng dẫn từng bước chi tiết từ lúc vừa bật máy tính, áp dụng cho hệ thống **AGIBOT A2 Ultra**.

---

## 0. Thông Số Mạng Cố Định & Phiên Bản
- **Phiên bản mã nguồn (Git Commit):** `fix/armor-paths-nudge` (Base SHA: `bdb409e`)
- **Máy trạm Teleop (Laptop):** `192.168.2.75`
- **Máy tính nhúng Robot (Orin):** `192.168.2.50` *(SSH user: `agi`, mật khẩu: `1`)*
- **Địa chỉ giao diện Web:** `https://192.168.2.75:8012`
- **Cổng UDP nhận lệnh từ Laptop sang Orin:** `5005`
- **Cổng UDP phản hồi feedback từ Orin về Laptop:** `5006`

---

## TÓM TẮT CHẠY NHANH (cập nhật 17/11)

Receiver stream vị trí khớp tới robot (action 402) ở 400 Hz. **Mặc định từ 17/11 là cấu hình NHANH (bám 1:1 trước, giảm rung sau):** tốc độ mức 6 cả hai bên, bộ đệm 30 ms, lọc thông thấp trên Orin TẮT, laptop `--smooth 1` (không lọc đầu ra thứ hai). Rung mạnh hơn cấu hình êm 16/11; cấu hình êm cũ: `PLAYOUT_MS=50 PACER_SMOOTH_MS=60 python arm_teleop_rx.py --speed 3` + `python main.py --speed 3 --smooth 3`. Giới hạn gia tốc có sẵn nhưng tắt: nó làm tay đung đưa khi dừng (16/11).

```bash
# --- Laptop, khi code receiver đổi (mã băm ở BƯỚC 0) ---
cd ~/a2_ws && scp scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py tools/cmd_recorder.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/

# --- Orin (ssh agi@192.168.2.50) ---
aima em load-env
bash /agibot/data/home/agi/Desktop/motion_player/disable_motion_player.sh
cd /agibot/data/home/agi/Desktop/mc
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml
python SetAction.py      # nhập 10 (RL_JOINT_DEFAULT), chờ robot đứng vững
python SetAction.py      # nhập 12 (RL_LOCOMOTION_ARM_EXT_JOINT_SERVO)
sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40   # cho phép tiến trình phát lệnh chạy real-time (mỗi terminal mới phải gõ lại)
# (tuỳ chọn) PACER_CPU=11 python arm_teleop_rx.py  -> ghim pacer vào lõi 11 nếu lõi đó rảnh (xem: top, nhấn 1)
python arm_teleop_rx.py  # phải in: lvl=6 và dòng "[pacer] arm command pacer running: 400 Hz, playout 30 ms, ... per-joint smoothing 0 ms x2, accel guard 0.00 ..., SCHED_FIFO 30 (real-time)"
# Nếu dòng [pacer] KHÔNG có "SCHED_FIFO 30" (có WARNING): Ctrl+C, gõ lại lệnh sudo prlimit ở ĐÚNG terminal này rồi chạy lại.

# --- Laptop ---
cd ~/a2_ws && source venv/bin/activate && python main.py
# Quest: mở https://192.168.2.75:8012 -> ENTER VR -> chờ ENGAGED
# Mặc định 17/11: --speed 6 --smooth 1 (nhanh, bám 1:1). Êm hơn, trễ hơn: python main.py --speed 3 --smooth 3 (receiver cũng --speed 3).
```

Kiểm tra nhanh khi chạy (thay đổi 16/11 tối, chưa xác nhận trên robot):
- Giao diện **không** hiện "ORIN PACER NOT REAL-TIME" / "ORIN PACER GAPS". Có thì xem lại `sudo prlimit` hoặc Orin đang quá tải.
- Bấm Home rồi bóp trigger kéo tay: **cổ tay đứng yên** (trước đây tự quay ~17°).
- Kéo chéo chậm: tay đi theo ngay, không khựng chờ chọn trục.
- Nhả trigger / để tay cầm yên: tay robot không tự trôi, khuỷu không tự xoay.
- Muốn đo rung / trễ: xem mục **QUY TRÌNH ĐO RUNG VÀ TRỄ** cuối tài liệu.

### SỰ CỐ KHUỶU 17/11 – nguyên nhân, cách nhận biết, cách khôi phục

- **Diễn biến (từ log):**
  - 10/11 – trưa 17/11: cả hai khuỷu bám lệnh tốt (~150 phiên).
  - 12:22: lần đầu chạy cấu hình D (`JOINT_AMAX`), khuỷu trái bị đập tới −1.98 rad.
  - 13:18: khuỷu trái thôi chạy.
  - Từ 13:24: khuỷu phải thôi chạy (đứng ở −0.03 hoặc lắc tự do theo vai, lệch lệnh 42–84 % mọi phiên sau đó).
- **Nguyên nhân (lỗi của bản pacer 12:00–14:00):**
  1. Robot đứng sau SetAction với khuỷu ±0.035. Giới hạn cũ ±0.10 ép lệnh nhảy 0.065 rad trong một bản tin (= 26 rad/s).
  2. Bộ giới hạn gia tốc riêng từng khớp coi đó là tốc độ thật của khớp, nên lái khuỷu lên **1.88 rad trong 0.5 s ở ~7 rad/s** (gấp > 2 lần giới hạn 3 rad/s của hãng). Mô phỏng lại đúng code lúc đó cho ra y như vậy.
  3. Sau cú đập đó, motor khuỷu không còn được MC điều khiển.
- **Đã sửa (pacer `orin_teleop_core.py` từ 45b9a4c6):**
  - Giới hạn khuỷu = giới hạn hãng ±0.03.
  - Lúc engage, giới hạn được nới để bao tư thế robot đang đứng.
  - Mỗi bản tin đổi ≤ 0.018 rad kể cả khi bị ép vào giới hạn.
  - Tốc độ nội bộ và trường vận tốc gửi đi bị chặn ở tốc độ cho phép (≤ 3 rad/s).
  - Phần dịch do ép vào giới hạn không tính là tốc độ.
  - Kiểm tra bằng SIL A13 và mô phỏng trường hợp xấu nhất: khuỷu được đưa từ từ vào 0.10, không vọt.
- **Nhận biết:**
  - Giao diện / terminal laptop hiện `JOINT NOT FOLLOWING: R J4 (cmd …, robot …)`: khớp lệch lệnh > 0.35 rad quá 1 s.
  - Robot đi lạ, phải xoay tay mới đi đúng hướng, bàn tay chạm chân.
- **Khôi phục:**
  1. **Dừng teleop tay đó.**
  2. Tắt nguồn robot hẳn, bật lại, SetAction 10 → 12.
  3. Kiểm tra: `python3 joint_limit_sweep.py --joints 4 --speeds 0.4 --strokes 2`. Phải in `all 14 joints follow the command` và khuỷu gập/duỗi.
  4. Nếu vẫn `FATAL … RJ4`: motor đang lỗi, cần AimMaster hoặc AgiBot xem / xoá mã lỗi.

### Không còn tự di chuyển khi vừa chạy main.py (17/11)

Trước đây robot sau SetAction đứng với khuỷu ±0.035 rad, ngoài giới hạn ±0.10 của mình, nên lệnh đầu tiên sau engage tự gập khuỷu 0.065 rad. Giờ giới hạn khuỷu = giới hạn hãng ±0.03, và lúc engage cả laptop lẫn receiver nới giới hạn để bao tư thế robot đang đứng (không vượt giới hạn AimDK): robot đứng yên đúng chỗ cho tới khi bạn điều khiển. SIL A13 kiểm tra trường hợp này.

### Cảnh tập gắp vật ảo trên robot thật (17/11)

`python main.py --scene pick_bin` (laptop, các bước khác giữ nguyên). Bàn cao 0,78 m, thùng và 4 vật **ảo** hiện quanh mô hình robot trên 8012 / Quest; nắm tay quanh vật ảo (≥ 45 %) để nhặt, mở (≤ 30 %) để thả vào thùng. Không ảnh hưởng lệnh gửi robot. Muốn khớp với bàn thật thì đặt bàn thật cao 0,78 m, mép trước cách robot 0,20 m.

### Chọn cấu hình giữa rung và trễ (17/11)

Trên robot 17/11: cấu hình mặc định (nhanh) bám kịp nhưng rung quá; cấu hình êm thì êm nhưng trễ quá. Các cấu hình A/B/C nằm giữa, **chưa chạy trên robot**. Thử **B trước**: còn rung thì sang C, còn trễ thì sang A.

| | Orin | Laptop | Trễ ước tính |
|---|---|---|---|
| Nhanh (cũ, trước D) | `JOINT_VMAX=off JOINT_AMAX=off python arm_teleop_rx.py` | `python main.py` | ~165 ms |
| **A** (hơi êm hơn) | `PLAYOUT_MS=50 PACER_SMOOTH_MS=20 python arm_teleop_rx.py` | `python main.py` | ~220 ms |
| **B** (giữa) | `PLAYOUT_MS=50 PACER_SMOOTH_MS=30 python arm_teleop_rx.py --speed 5` | `DIRECT_OUT_FIR2=4 python main.py --speed 5 --smooth 2` | ~270 ms |
| **C** (êm hơn) | `PLAYOUT_MS=50 PACER_SMOOTH_MS=40 python arm_teleop_rx.py --speed 4` | `python main.py --speed 4 --smooth 3` | ~320 ms |
| Êm (cũ, 16/11) | `PLAYOUT_MS=50 PACER_SMOOTH_MS=60 python arm_teleop_rx.py --speed 3` | `python main.py --speed 3 --smooth 3` | ~370 ms, thêm chậm khi vung tay xa |
| **D – MẶC ĐỊNH từ 17/11** (riêng từng khớp) | `python arm_teleop_rx.py` (= `JOINT_VMAX=1.5,1,2,0.8,2,2,2 JOINT_AMAX=6`, operator xác nhận êm 17/11 19:10; D cũ = `1.2,1.5,1.2,1.5,2,2,2`) | `python main.py` | Orin +70–80 ms so với Nhanh (mô phỏng) |
| **D1** (D nhanh hơn) | `JOINT_AMAX=8 python arm_teleop_rx.py` | `python main.py` | Orin +50–60 ms so với Nhanh (mô phỏng) |

- Trễ là ước tính cộng từng chặng (laptop + bộ đệm + lọc Orin + ~80 ms của MC), chưa đo trên robot.
- `--speed` hai bên **phải giống nhau**. Đổi cấu hình: nhả trigger, Ctrl+C receiver, chạy lại receiver rồi `main.py`; terminal mới thì gõ lại `sudo prlimit ...` trước.
- Vì sao chọn như vậy: LPF trên Orin 20–30 ms đã lấy phần lớn độ êm (30 → 60 ms gần như chỉ thêm trễ); bộ đệm 50 ms chỉ tốn 20 ms mà chống giật do gói đến thành chùm; mức 5 vẫn bám ~90 % tay 1:1 nhưng gia tốc 4.0 rad/s² thay vì 6.28 (rung bị kích lúc tăng tốc / phanh).
- Khi B gần đúng ý, chỉ chỉnh **một núm**: `PACER_SMOOTH_MS` trên Orin (+10 ms = êm hơn, chậm thêm ~20–30 ms; -10 ms = ngược lại), laptop giữ nguyên.
- **D / D1 (giới hạn riêng từng khớp; D là mặc định từ 17/11, operator: "hơi rung nhẹ, hơi delay chút xíu, rất ổn")**: giữ `--speed 6` cả hai bên. 7 số là J1..J7 (vai pitch, vai roll, vai yaw, khuỷu, cẳng tay yaw, cổ tay pitch, cổ tay roll), dùng cho cả hai tay; 14 số = trái rồi phải; 1 số = mọi khớp.
  - `JOINT_VMAX`: tốc độ tối đa từng khớp (rad/s). 402 rung tỉ lệ với tốc độ khớp. Vai J1/J3 nặng và chạy nhanh nhất (tới 2.2 rad/s khi vung tay), nên chặn ở 1.2. Khớp gần bàn tay giữ 2.0. Cử động chậm, làm việc tỉ mỉ vẫn 1:1, chỉ cú vung nhanh bị chặn.
  - `JOINT_AMAX`: gia tốc tối đa từng khớp (rad/s²), bộ hãm biết trước điểm dừng, không vọt qua đích. **Bắt buộc đi kèm `JOINT_VMAX`**: chỉ chặn tốc độ thì vận tốc nhảy bậc lúc chạm / rời giới hạn (receiver in cảnh báo).
  - Mô phỏng trên quỹ đạo tay thật 17/11 (`venv/bin/python tools/filter_report.py logs/trace_laptop_<t>.jsonl --raw --speed 6 --configs none,joint-k3,joint-k1`):

    | | Rung lệnh (rad/s) | Gia tốc đỉnh (rad/s²) | Trễ thêm trên Orin | Vọt qua đích |
    |---|---|---|---|---|
    | Nhanh | 0.084–0.133 | 107–185 | 30–40 ms | 0 |
    | D (`AMAX=6`) | 0.026–0.042 | 6.0 (giới hạn hãng 6.28) | 100–120 ms | ≤ 5 mrad |
    | D1 (`AMAX=8`) | 0.033–0.053 | 8.0 | 80–100 ms | 0 |
    | Êm cũ (LPF 60, mức 3) | 0.029 | 30–41 | 210–270 ms | tới 71 mrad |

  - Còn rung: hạ vai `JOINT_VMAX=1.0,1.2,1.0,1.2,1.5,2,2`. Còn trễ: dùng D1, hoặc nâng vai lên 1.5.
  - Rung đo trên robot 16/11 (mức 3): cổ tay J6/J7 rung gấp ~20 lần lệnh của chính nó (lệnh cổ tay đã mượt). Nghĩa là rung cổ tay đến từ vai / khuỷu truyền xuống. Hạ tốc độ cổ tay vô ích, hạ tốc độ vai mới giảm được.
- Chạy kèm `cmd_recorder.py` (mục **QUY TRÌNH ĐO RUNG VÀ TRỄ**) để biết rung còn lại do lệnh gồ ghề (lọc thêm có ích) hay do robot (chỉ còn cách hạ gia tốc).

Mặc định hiện tại (không cần gõ thêm tham số):

| Hạng mục | Mặc định | Đổi bằng |
|---|---|---|
| Tốc độ khớp | **mức 6 = 2.0 rad/s, 6.28 rad/s² (mặc định từ 17/11, bám 1:1)**; mức 3 = 0.8 rad/s êm nhất nhưng trễ xa (16/11: mức 4 rung nhẹ, mức 5 bàn tay lắc quanh cổ tay, mức 6 rung mạnh), hai bên phải cùng mức | `--speed N` ở cả `main.py` và `arm_teleop_rx.py` |
| Tần số phát lệnh trên Orin | 400 Hz (ít rung nhất đã thử) | `PACER_HZ=200 python arm_teleop_rx.py` |
| Ưu tiên tiến trình phát lệnh (pacer) trên Orin | real-time `SCHED_FIFO 30`, cần `sudo prlimit ...` trước receiver; thiếu bước này Orin bỏ đói pacer, lệnh cách nhau tới 140 ms = tay giật (16/11) | `PACER_CPU=11` (ghim vào một lõi rảnh) |
| Bộ đệm khử jitter trên Orin | 30 ms (từ 17/11) | `PLAYOUT_MS=50` (thêm 20 ms trễ, chịu gói đến thành chùm tốt hơn) |
| Định dạng lệnh khớp | vị trí + vận tốc thật | `ARM_CMD_FORMAT=vendor` (velocity / effort = 0 như ví dụ của hãng) |
| Lọc thông thấp trên Orin (khuyến nghị của hãng) | **tắt (từ 17/11)**. Bật 2 tầng 60 ms (cùng bộ đệm 50 ms): lệnh bớt gồ ghề khoảng 4 lần, gia tốc đỉnh 40 → 5,6 rad/s², nhưng trễ thêm ~140 ms ([docs/FILTER_REPORT.md](docs/FILTER_REPORT.md)) | `PACER_SMOOTH_MS=20` / `30` / `60`; `PACER_FILTER=kalman` (Kalman vận tốc không đổi, thêm `KALMAN_LPF_MS=30` để lọc tiếp) / `oneeuro` |
| Làm mượt trên laptop | **`--smooth 1`** (từ 17/11: gia tốc ≤ 6.0, đáp ứng 0.04 s, lọc đầu ra 4 tick, không có lớp thứ hai) | `--smooth 3` (6 + 4 tick, êm hơn, trễ hơn ~60–80 ms) |
| Giới hạn gia tốc trên Orin | **tắt**: làm lệnh dao động khi dừng = tay đung đưa (16/11). Luôn bật: cảnh báo khi hai lệnh cách nhau > 30 ms | `PACER_MAX_ACC=6.28`; riêng từng khớp: `JOINT_AMAX` (xem cấu hình D) |
| Giới hạn riêng từng khớp trên Orin | **bật, cấu hình D (từ 17/11)**: tốc độ 1.5/1/2/0.8/2/2/2 rad/s (J1..J7; chọn lại 17/11 tối theo mức rung của từng khớp, operator xác nhận êm; D cũ 1.2/1.5/1.2/1.5/2/2/2), gia tốc 6 rad/s² | `JOINT_VMAX=off JOINT_AMAX=off` = tắt (như cấu hình Nhanh cũ);  `JOINT_VMAX`, `JOINT_AMAX`, `JOINT_TAU_MS` (7 số J1..J7, xem cấu hình D) |
| Ngón tay | nắm hết hành trình 0..2000. **ngón cái nghỉ ở tư thế cầm vật, đóng/mở theo cần gạt (từ 17/11)**: xoay 400, gập 900 (raw; 1200 → 800 để không chạm ngón trỏ, tối 17/11 → 400 kèm lực giữ ngón cái 2280 để bớt lắc khi vung tay; `THUMB_EFFORT=1140` = lực như cũ), kể cả lúc tay mở / chưa engage; tay mở thì ngón cái ở tư thế này, nắm thì ngón cái gập thêm cùng 4 ngón (xoay 400 → 800, gập 900 → ~1500 khi nắm hết, để ngón trỏ / giữa vẫn nắm hết được) | `--thumb-pose SWING,FLEX` ở cả hai bên (vd `1400,1100` = ngón cái áp sâu hơn); `--thumb-pose off` = ngón cái theo tay như cũ (khi đó `--thumb-rest` có tác dụng); `--hand-raw-max` |
| Khuỷu tay | ưu tiên mở ra ngoài khi gần thân hơn 15 cm; HOME giữ nguyên | `--elbow-out 9` (nhẹ) / `0` (tắt) |
| Nút dịch XYZ / xoay | đích trượt mượt, không nhảy từng bước | `NUDGE_GLIDE=0` |
| Kéo chuột tịnh tiến / nút dịch | cổ tay giữ thẳng, bàn tay xoay theo cẳng tay | `SOFT_ORI=0` |
| Sau khi ngừng kéo chuột | khuỷu có thể tự xoay ra thêm 1-3 s, bàn tay giữ nguyên (tay cầm Quest: không còn, xem dòng "Tay cầm đứng yên") | `IK_SETTLE_S=0` |
| Nhả trigger khi robot còn đang đuổi | tay đi tiếp theo đường thẳng tới đích | `XR_COAST=0` |
| Nút Home / preset | đi tối đa 0.8 rad/s (bằng mức 3) cả khi chạy mức cao hơn | `HOME_MAX_VEL=2.0` (về nhanh theo `--speed`) |
| Khóa trục tay cầm | **mềm, nới lỏng 17/11**: đưa tay thật thẳng theo một trục (lệch < 12°, nhả khi lệch > 20°) thì robot đi theo trục đó, phần lệch trục vẫn đi được 30 %; tay chậm (< ~3 cm/s, ~10°/s khi xoay) quá 0,25 s thì nhả về tự do 3D để chỉnh tỉ mỉ; đưa chéo / mới bắt đầu thì tự do 3D. Trước 17/11: 20°/30°, phần lệch trục bị bỏ hẳn, khóa giữ suốt cả lúc chỉnh chậm (chỉnh 1 cm sau một lần kéo thẳng = robot không nhúc nhích). Mô phỏng, chưa chạy trên robot | `--no-axis-lock` (luôn tự do); chỉnh độ chặt: `AXLOCK_SNAP_DEG=20 AXLOCK_RELEASE_DEG=30 AXLOCK_LEAK=0 AXLOCK_SLOW_FREE_S=0` = như cũ |
| Cổ tay khi chỉ tịnh tiến (chỉ giữ trigger, kéo chuột, nút dịch) | **khóa cứng** J6/J7 ở góc hiện tại; bàn tay đặt bằng J1-J5. Trước đây cổ tay vẫn bị lệnh quay (rung cổ tay) và sau Home tự quay ~17° ở lần bóp trigger kế tiếp (16/11) | `WRIST_FREEZE_SOFT=0` |
| Cảnh báo khớp không theo lệnh | laptop hiện `JOINT NOT FOLLOWING: <tay> J<k> (cmd …, robot …)` khi một khớp lệch lệnh > 0.35 rad quá 1 s (MC không còn điều khiển khớp đó, xem SỰ CỐ KHUỶU 17/11) | chỉ cảnh báo; dừng teleop, tắt / bật nguồn robot |
| Cảnh báo pacer trên giao diện | Orin gửi về trạng thái pacer; laptop hiện "ORIN PACER NOT REAL-TIME" nếu quên `sudo prlimit`, hoặc "ORIN PACER GAPS" nếu có ≥ 3 khoảng hở > 30 ms trong 30 s | chỉ cảnh báo, không tự xử lý |
| Robot đuổi theo tay khi bị tụt lại | đích IK đặt trước bàn tay robot 10 cm và bộ giới hạn không phanh ở các điểm trung gian, chỉ phanh khi gần đích thật (16/11) | `XR_LEAD_POS_M=0.06` (cũ), `XR_FAR_CRUISE=0` (phanh như cũ) |
| Tay cầm đứng yên hoặc đã nhả | khớp giữ nguyên: không giải lại IK khi đích không đổi và bàn tay đã ở đích, nên khuỷu / cẳng tay không tự xoay nữa (16/11). Tư thế chỉ điều chỉnh trong lúc bàn tay đang di chuyển | `XR_HOLD_STILL=0` (như cũ: khuỷu tự xoay ra 1-3 s sau khi dừng) |
| Bảng chẩn đoán XR trong kính | ẩn; chỉ hiện khi có lỗi XR. Bấm ENTER VR hai lần không còn báo lỗi | mở trang với `?xrdiag=1` để luôn hiện |
| Tay cầm, chỉ giữ trigger (tịnh tiến) | cổ tay giữ thẳng, bàn tay xoay theo cẳng tay; giữ thêm grip thì hướng bàn tay được giữ đúng | `XR_SOFT_ORI=0` |
| Ngón tay khi cánh tay đang chạy | giữ nguyên lệnh; nắm / mở được áp dụng khi tay chậm dưới 0.15 rad/s | `HAND_HOLD_MOVING=0` |
| Cách dừng | `--smooth 1` (mặc định): không có lọc đầu ra thứ hai (`DIRECT_OUT_FIR2=1`); `--smooth 3`: phanh tăng dần / giảm dần `DIRECT_OUT_FIR2=4` (17/11, trước 10; tốc độ tối đa giữ nguyên) | `DIRECT_OUT_FIR2=4` / `10` (êm hơn, trễ hơn) |

Tình trạng kiểm chứng (16/11):

| Hạng mục | Đã xác nhận trên robot thật | Mới kiểm bằng mô phỏng |
|---|---|---|
| Phát 400 Hz, tốc độ mức 3, pacer real-time | không giật, trễ khoảng 0,5–1 s với cử động lớn (16/11) | |
| Tốc độ mức 4 | rung nhẹ (16/11) | |
| Tốc độ mức 5 | nhanh hơn nhưng bàn tay lắc quanh cổ tay (16/11) | |
| Tốc độ mức 6 | rung mạnh, quá nhanh (người vận hành, 16/11) | |
| Ngón tay nắm hết hành trình, ngón cái nghỉ 500 | | chưa có phản hồi sau khi thử |
| Khuỷu mở ra ngoài | đã sửa được (người vận hành xác nhận) | |
| Nút dịch trượt mượt | êm (người vận hành xác nhận) | |
| Phanh tăng dần, Home chậm, cổ tay thẳng khi tịnh tiến bằng tay cầm, giữ ngón tay khi tay chạy | | chưa xác nhận |
| Khóa trục mềm, đích IK 10 cm + không phanh ở điểm trung gian | | mô phỏng lại trên log robot 16/11: robot tới đích sau khi tay dừng 0,9 → 0,4 s (trung vị); chưa chạy trên robot |
| Giữ yên khớp khi tay cầm đứng yên / đã nhả | | thử trong mô phỏng: sau khi nhả, khớp tự xoay 0,12–0,19 rad → 0; chưa chạy trên robot |
| Khóa cổ tay khi tịnh tiến, khóa trục bắt đầu tự do, cảnh báo pacer | | mô phỏng + SIL 57/57; chưa chạy trên robot |
| Lọc đầu ra laptop 6 + 4 tick (bớt ~60 ms trễ, 17/11) | | phân tích log 16/11 + SIL 38/38; chưa chạy trên robot |
| Ẩn bảng chẩn đoán XR / lỗi ENTER VR hai lần | | chưa thử trên Quest |

Tay rung hoặc giật: xem `docs/TROUBLESHOOTING.md` mục 1b.

---

## BƯỚC 0: ĐỒNG BỘ CODE SANG ROBOT (NẾU CÓ CẬP NHẬT)

Mở terminal trên Laptop và chạy lệnh:
```bash
# Kiểm tra mã băm SHA256 trước khi gửi:
sha256sum scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py
# Kì vọng arm_teleop_rx.py:   ce6f9b97f8a72622761b7f1ded44797b2610548df4372b08f70d0df00aaa47b9   (17/11: ngón cái nghỉ tư thế cầm vật --thumb-pose)
# Kì vọng orin_teleop_core.py: 9ca0832e17b03af26c89e76dee45e600e7bf19692273f0e4ed978ae0f720747c   (17/11 chiều: khuỷu ±0.03, giữ tư thế lúc engage, trần 0.018 rad/bản tin + chặn tốc độ -> chép lại CẢ HAI file)
# tools/cmd_recorder.py cũng đổi 16/11 tối (hàng đợi 20 -> 200, hết mất bản tin khi ghi) -> chép lại.

# Đồng bộ sang Orin (4 tệp receiver + bộ ghi trace vào thư mục mc):
scp scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py tools/cmd_recorder.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/
```

---

## BƯỚC 1: KHỞI ĐỘNG TELEOP SERVER TRÊN LAPTOP

Mở **Terminal 1** trên Laptop:

```bash
cd ~/a2_ws
source venv/bin/activate

# Lệnh tiêu chuẩn cực kỳ đơn giản (mặc định: real-udp, tay cầm Quest bật, --speed 6 = 2.0 rad/s,
#   --limiter direct, khóa trục mềm, scale 1:1, --smooth 1, --elbow-out 15, ngón tay 0..2000, ngón cái --thumb-pose 400,900,800,1500):
python main.py

# Khuỷu tay: mặc định ưu tiên mở ra ngoài (--elbow-out 15, đơn vị cm; 9 = nhẹ hơn, không đổi tư thế kiểu HOME; 0 = tắt).
# Sau khi ngừng kéo chuột / nhả trigger, khuỷu có thể tự xoay ra thêm trong 1-3 s, bàn tay giữ nguyên vị trí (tắt: IK_SETTLE_S=0).
# Nhả trigger khi robot còn đang đuổi theo: tay đi tiếp theo đường thẳng tới đích (tắt: XR_COAST=0 python main.py).
# Nút dịch XYZ / xoay trên bảng web: đích trượt mượt tới vị trí mới thay vì nhảy từng bước (tắt: NUDGE_GLIDE=0).
# Kéo chuột tịnh tiến / nút dịch: bàn tay xoay theo cẳng tay để cổ tay giữ thẳng (tắt: SOFT_ORI=0). Tay cầm Quest không bị ảnh hưởng.
# Nghi thay đổi phía laptop gây giật -> tắt hết để so sánh, rồi bật lại từng cái:
#   DIRECT_OUT_FIR2=1 XR_SOFT_ORI=0 HAND_HOLD_MOVING=0 SOFT_ORI=0 IK_SETTLE_S=0 XR_COAST=0 NUDGE_GLIDE=0 python main.py --elbow-out 0
# Tùy chọn thay đổi tốc độ bằng --speed N (1..6, mặc định 6):
# Level 1: 0.4 rad/s, 1.0 rad/s² (an toàn khởi động)
# Level 2: 0.6 rad/s, 1.5 rad/s²
# Level 3: 0.8 rad/s, 2.0 rad/s² (theo kịp ~62% chuyển động tay 1:1, êm nhất đã thử; mặc định trước 17/11)
# Level 4: 1.0 rad/s, 2.5 rad/s² (theo kịp ~79%)
# Level 5: 1.5 rad/s, 4.0 rad/s² (theo kịp ~90%)
# Level 6: 2.0 rad/s, 6.28 rad/s² (MẶC ĐỊNH từ 17/11; sát giới hạn firmware 3.0 rad/s / 6.28 rad/s²; rung mạnh nhất)
# python main.py --speed 2

# Tùy chọn đổi khỏi mặc định (LẦN ĐẦU trên robot thật xem "Trình tự lần đầu" ở BƯỚC 4):
#   --limiter brake  : luật giới hạn cũ (giới hạn cả gia tốc giảm tốc); TRỄ NHIỀU: đo trên robot thật 12/11 là
#                      0.42-0.78 s từ tay người tới tay robot. Chỉ dùng cho lượt kiểm tra an toàn đầu tiên.
#   --smooth 3 / 2 / 0: làm mượt lệnh (gia tốc tỉ lệ + lọc trung bình trượt): 3 = êm nhất; 2 = vừa; 1 = MẶC ĐỊNH (17/11), nhẹ; 0 = tắt: dừng gắt
#   --no-axis-lock   : tắt khóa trục mềm (mặc định 17/11: đi thật thẳng theo một trục, lệch < 12°, thì khóa trục đó, lệch trục vẫn đi 30 %; tay chậm hoặc đi chéo thì tự do 3D)
#   --xr-scale 0.6   : tỉ lệ tay cầm -> robot lúc khởi động (mặc định 1.0 = 1:1). Đổi khi đang chạy: thanh "Controller -> Robot Scale" trên bảng web

# Chạy mô phỏng MuJoCo thuần:
# python main.py --sim
```

**Để robot bám tay ít trễ nhất:**
- Chạy đúng `python main.py` (KHÔNG thêm `--limiter brake`, KHÔNG thêm `--xr-scale 1.0`). Phần trễ còn lại ~0.15 s là của mạng + Orin + MC, không giảm được từ laptop.
- Đưa tay vừa phải: ở mức 3 bàn tay robot đi được khoảng 0.25 m/s (đo 16/11), trong khi tay người thường vung 0.3-0.7 m/s. Vung nhanh hơn thì khớp chạy kịch trần tốc độ, robot bị bỏ lại và tới đích muộn hơn (biên độ càng lớn càng muộn). Hạ scale (thanh "Controller -> Robot Scale") làm robot theo kịp hơn nhưng không còn 1:1.
- Từ HOME, tay robot đã gần duỗi hết (0.494/0.50 m): **kéo tay vào gần thân hoặc nâng lên trước**, rồi mới đưa ra ngoài/xuống dưới; nếu không tay sẽ đứng yên ở biên tầm với (Terminal 1 in `[REACH] Target out of reach`).
- Không kéo bàn tay robot băng qua trước ngực hoặc co sát vai: bộ chống va chạm sẽ chặn đứng tay (Terminal 1 in `[ARMOR] BLOCKED`).
- Mặc định hai bên là mức 6 từ 17/11 (2.0 rad/s, 6.28 rad/s²; trước đó mức 3 = 0.8 rad/s, 2.0 rad/s²) và receiver phát lệnh 400 Hz. Trên robot 16/11: mức 4 rung nhẹ, mức 5 bàn tay lắc quanh cổ tay, mức 6 rung mạnh. Muốn êm hơn thì `--speed 5`/`4`/`3` ở CẢ HAI bên, đổi lại chậm hơn (mức 3 theo kịp ~62% chuyển động tay 1:1, mức 4 ~79%, mức 5 ~90%, mức 6 ~95%).
- Khi tay đứng yên bất thường: ghi lại giờ, gửi `logs/trace_laptop_*.jsonl` (đã có bản ghi `DIAG` 20 Hz: lý do chặn, hết tầm với, vị trí tay cầm).

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
sha256sum arm_teleop_rx.py orin_teleop_core.py
# Kỳ vọng đúng mã: arm_teleop_rx.py ce6f9b97f8a72622761b7f1ded44797b2610548df4372b08f70d0df00aaa47b9
#                  orin_teleop_core.py 9ca0832e17b03af26c89e76dee45e600e7bf19692273f0e4ed978ae0f720747c

# 4. Nạp cấu hình ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

# 5. CHUYỂN ACTION THỦ CÔNG (BẮT BUỘC TRƯỚC KHI BẬT LIVE). Chọn theo TÊN trong menu, số có thể đổi theo firmware.
#
#   Luồng A - Action 12 (RL_LOCOMOTION_ARM_EXT_JOINT_SERVO / 402) - MẶC ĐỊNH cho teleop tay:
python SetAction.py          # nhập 10 (RL_JOINT_DEFAULT), chờ robot đứng vững (~1 s)
python SetAction.py          # nhập 12 (RL_LOCOMOTION_ARM_EXT_JOINT_SERVO, tức 402)
#
#   Luồng B - Action 07 (PASSIVE_UPPER_BODY_JOINT_SERVO) - lưu ý phải qua bước 2 trước rồi mới đến 7:
python SetAction.py          # nhập 2 (STAND / STAND_DEFAULT), chờ robot đứng ổn định
python SetAction.py          # nhập 7 hoặc 07 (PASSIVE_UPPER_BODY_JOINT_SERVO)
# Sau Luồng B, arm_teleop_rx.py chấp nhận action này (PASSIVE_UPPER_BODY_JOINT_SERVO trong whitelist).
#
# Sau bước này robot ở chế độ servo tay ngoài; receiver kiểm tra GetAction và từ chối nếu chưa đúng.
# Chọn theo TÊN trong menu (số có thể đổi theo firmware). MC trả "called successfully" cả khi bỏ qua yêu cầu.
#
# --- XU LY LOI: "FATAL: Robot action is 'McAction_DEFAULT'" ---
# Nghia la buoc 5 chua duoc chay hoac MC da reset ve DEFAULT sau mot loi.
# Khac phuc:
#   a) Chay lai SetAction theo Luong A hoac Luong B:
#      - Neu chay Luong B: bat buoc phai goi Action 2 truoc roi moi goi Action 7.
#   b) Neu robot khong chuyen action (MC tra "called successfully" nhung GetAction van la DEFAULT):
#      - Tat nguon robot hoan toan, bat lai, doi 30 s.
#      - Chay lai SetAction tu buoc 10 -> 12 hoac 2 -> 7.
#   c) Neu muon receiver tu dong goi SetAction (nguoi van hanh + E-stop san sang):
python arm_teleop_rx.py --auto-set-action 7   # tu dong SetAction(7) neu chua dung action
python arm_teleop_rx.py --auto-set-action 12  # tu dong SetAction(12) neu chua dung action
#      Receiver van xac nhan lai sau khi goi; thoat neu MC khong chap nhan trong 8 s.
# -------------------------------------------------------------------------

# 6. Cho phép tiến trình phát lệnh chạy real-time (BẮT BUỘC, mỗi terminal mới gõ lại; chỉ đổi giới hạn của terminal này,
#    đóng terminal là hết, không đổi cấu hình robot). Thiếu bước này tay robot giật (Orin bỏ đói pacer, 16/11).
sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40

# 7. Khởi động receiver (mặc định LIVE, chế độ stream 402, IP 192.168.2.75, cổng 5005, --speed 6, thumb swing bật)
python arm_teleop_rx.py
# Bàn tay: hành trình ngón tối đa 2000/2000 (trước đây bị chặn 800 = 40%); ngón cái mặc định NGHỈ ở tư thế cầm vật (400/900) kể cả khi chưa engage /
# mất kết nối; khi nắm, laptop gập ngón cái tới 1500 (xoay tới 800; main.py --thumb-pose 400,900,800,1500). Đổi tư thế nghỉ: --thumb-pose 1400,1100 (cả hai bên);
# --thumb-pose off --thumb-rest 300 = ngón cái theo tay như cũ, nghỉ ở 300. --hand-raw-max 1400 = hạ hành trình.
# (main.py có cùng các tham số --hand-raw-max / --thumb-pose / --thumb-rest; nên đặt hai bên giống nhau.)
# Phải thấy 2 dòng: "[pacer] arm command pacer running: 400 Hz ..." và "arm commands are paced by a separate light process".
# Mặc định là 400 Hz (đổi 13/11: ít rung nhất trên robot thật), không cần gõ PACER_HZ. Dòng [pacer] phải ghi "400 Hz"; nếu ghi 100 hay 200 Hz thì file trên Orin là bản cũ, chép lại.
# Dòng [pacer] mặc định ghi "400 Hz, playout 30 ms, pid <số>, message format: position + velocity, per-joint smoothing 0 ms x2 (= tắt, 17/11), accel guard 0.00 rad/s^2 (both directions, speed <= 2.00 rad/s), SCHED_FIFO 30 (real-time)".
# Mặc định (cấu hình D, từ 17/11) có thêm dòng "[pacer] per-joint limits (right arm J1..J7; ...): speed cap rad/s 1.2/1.5/..., accel rad/s^2 6/...".
# Receiver khựng > 0.5 s: pacer GIỮ tay, in "[pacer] parent silent ... ms -> HOLD arm" rồi "... -> resumed"; chỉ thoát sau 3 s im lặng hoặc khi receiver chết.
# Pacer dừng vì bất kỳ lý do gì luôn in "[pacer] stopped: <lý do>" (kèm traceback nếu crash); receiver báo FAULT "pacer process stopped (exit code N, ...)" hoặc "pacer process frozen > 0.5 s".
# Không có "SCHED_FIFO 30" mà có WARNING "not real-time": chưa chạy bước 6 trong terminal này -> Ctrl+C, làm bước 6, chạy lại.
# Nếu vẫn in "[pacer] WARNING: ... gap(s) > 30 ms between commands" dù đã real-time: thử ghim lõi, PACER_CPU=11 python arm_teleop_rx.py (lõi rảnh nhất theo top, nhấn 1).
# Tuỳ chọn: PACER_SMOOTH_MS=20/30/60 / PLAYOUT_MS=50 (êm hơn, trễ hơn) | PACER_FILTER=kalman (+ KALMAN_LPF_MS=30) | --speed 2 ở CẢ main.py (êm hơn nữa) | PACER_FILTER=oneeuro | ARM_CMD_FORMAT=vendor | PACER_MAX_ACC=6.28 (KHÔNG khuyên: tay đung đưa khi dừng)
# (Phát nhanh hơn chu kỳ 100 Hz của MC để mỗi chu kỳ MC luôn có điểm mới: 100 Hz rung nhiều nhất, 200 Hz đỡ rõ, 400 Hz ít nhất.)
# (Tiến trình con siêu nhẹ giữ nhịp phát lệnh 400 Hz, chạy real-time nhờ bước 6. Tắt: ORIN_PACER=0 python arm_teleop_rx.py)
# python arm_teleop_rx.py --speed 5     # khi main.py cũng chạy --speed 5 (hai bên phải cùng mức; mặc định 6)

# Tùy chọn chạy dry-run thử nghiệm không xuất xung:
# python arm_teleop_rx.py --dry
```

### Bảng Tốc Độ (`--speed N`) - Chu kỳ phát 100 Hz (dt = 0.010 s)
| Level | Max Vel (rad/s) | Max Accel (rad/s²) | Max Step (rad/tick @ 100Hz) | Ghi Chú |
|---|---|---|---|---|
| 1 | 0.40 | 1.00 | 0.004 | Mức an toàn khởi động, làm quen |
| 2 | 0.60 | 1.50 | 0.006 | Êm hơn nữa, chậm hơn |
| 3 | 0.80 | 2.00 | 0.008 | Êm nhất trong các mức đã thử 16/11 (mặc định trước 17/11); theo kịp ~62% chuyển động tay 1:1 |
| 4 | 1.00 | 2.50 | 0.010 | Theo kịp ~79%; rung nhẹ (16/11) |
| 5 | 1.50 | 4.00 | 0.015 | Theo kịp ~90%; bàn tay lắc quanh cổ tay (16/11) |
| 6 | 2.00 | 6.28 | 0.020 | **Mặc định từ 17/11**; theo kịp ~95% nhưng rung mạnh (16/11); sát giới hạn firmware, receiver kẹp mỗi bản tin ≤ 18 mrad |

> **Luật giới hạn:** mặc định là `--limiter direct` (bám gần như trực tiếp, trễ ~35-100 ms, các khớp của một tay chậm lại cùng tỉ lệ nên bàn tay đi thẳng), kèm `--smooth 1` (mặc định 17/11; `--smooth 3` = lọc đầu ra 6 + 4 tick, êm hơn). `--limiter brake` là luật cũ: phanh êm nhưng trễ 0.4-0.8 s, chỉ dùng cho lượt kiểm tra an toàn đầu tiên. Chỉ cần đặt ở laptop, receiver không đổi.
>
> **Mức tốc độ:** mặc định là mức 6 (từ 17/11) ở **cả** `main.py` lẫn `arm_teleop_rx.py`; đổi mức phải gõ `--speed N` ở cả hai bên. Mức 4, 5, 6 đã chạy trên robot thật (13/11, 16/11; mức 6 bám kịp nhưng rung mạnh, 16–17/11); khuỷu và nút dịch trượt mượt đã được người vận hành xác nhận; cổ tay thẳng khi tịnh tiến, giữ ngón tay, phanh tăng dần, Home chậm **chưa được xác nhận trên robot thật** (xem bảng Tình trạng kiểm chứng). SIL không mô phỏng việc MC thật từ chối bước nhảy lớn, nên lần đầu phải có người giữ E-stop và nên đi từng mức (4 → 5 → 6). Cổ tay vẫn dùng 50% gia tốc (`WRIST_SCALE=0.5`).

> **Kiểm tra tần số xuất lệnh:** Trên terminal Orin có thể kiểm tra tần số gửi bằng lệnh:
> ```bash
> ros2 topic hz /motion/control/arm_joint_command
> # Kỳ vọng: average rate ~400 Hz (pacer mặc định). Thấp hơn rõ (vd ~300 Hz) = pacer bị bỏ đói: kiểm tra bước 6 (prlimit)
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
  - **Tay cầm VR: ánh xạ 1:1 tuyệt đối (không dời neo):** Khi bóp cò/grip, tay robot đi đúng quãng đường và đúng chiều như tay người (tịnh tiến và xoay cùng hệ "nhập vai robot": đưa tay ra trước → tay robot ra phía trước của robot; chúc tay cầm lên → bàn tay robot chúc lên). Neo xoay lấy theo hướng bàn tay robot **thực tế** lúc bóp Grip. Khi kéo ra ngoài tầm với, mục tiêu bị giữ trên mặt cầu giới hạn nhưng điểm neo KHÔNG bị dời: robot đứng ở biên cho tới khi tay người quay lại vùng với tới được, rồi khớp lại đúng 1:1 (đưa tay về chỗ cũ → robot về đúng chỗ cũ). Lưu ý: ở tư thế HOME tay robot gần duỗi hết (EE cách vai 0.49/0.50 m) nên gần như không đi xuống/vươn ra thêm được; cổ tay bị giới hạn khớp J6 ±45°, J7 ±30° nên xoay lớn sẽ dừng ở giới hạn. Khi nhả ly hợp, chốt giữ mục tiêu tại chỗ (`freeze_arm_target`), bóp lại thì neo theo vị trí hiện tại, không giật.
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
>   - Giới hạn an toàn Teleop: Trần lệnh ngón tay $\le 2000$ (hết hành trình; hạ bằng `--hand-raw-max`, mô-men xoắn cố định 1140 ~ 20% lực cực đại).
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
   - Khi badge còn `SYNC` (receiver chưa chạy / mất feedback): thử nắm tay trên giao diện. Xác nhận bàn tay robot thật vẫn đứng yên. Hệ thống **tự Engage** ngay khi feedback tươi (không còn nút Engage) - từ lúc đó thanh trượt / cần analog điều khiển bàn tay thật.
3. **Kiểm tra 3 - Giữ Trung Tính Khớp Vung Ngón Cái (Thumb Swing):**
   - ngón cái nghỉ ở tư thế cầm vật, đóng/mở theo cần gạt: `thumb_0` (xoay) 400/2000, `thumb_1` (gập) 900/2000, cả khi tay mở và khi chưa engage (`--thumb-pose`; với `--thumb-pose off` thì nghỉ ở `--thumb-rest` 500/2000). Chạy receiver với `--no-thumb-swing` thì `thumb_0` luôn ở 0.
4. **Kiểm tra 4 - Thử Nghiệm Nắm Mở Từ Từ (Grasp Test):**
   - (Tự động Engage khi đã SYNC.) Di chuyển nhẹ ngón tay trên kính hoặc kéo thanh trượt đóng mở bàn tay trên web.
   - Quan sát các ngón tay robot co lại mượt mà, Close 100% kiểu cylinder gửi khoảng 1830-1880/2000 cho 4 ngón; ngón cái xoay từ 400 tới 800, gập từ 900 (tay mở) tới khoảng 1450–1500 (nắm hết) cùng 4 ngón, không chắn ngón trỏ / giữa.
   - Thử thả tay / nhả ly hợp: bàn tay dừng lại hoặc mở về vị trí an toàn.

---

## BƯỚC 4: KÍCH HOẠT VÀ ĐIỀU KHIỂN

### Cách A: Điều khiển bằng Kính Meta Quest 3 (VR)
1. Đeo kính Meta Quest (đã kết nối cùng mạng Wifi với Laptop).
2. Mở trình duyệt Browser trên kính, truy cập: `https://192.168.2.75:8012`
   *(Nếu thấy cảnh báo chứng chỉ SSL tự ký, chọn **Advanced** $\rightarrow$ **Proceed to 192.168.2.75**)*.
3. Bấm **ENTER VR**.
4. **Trình tự lần đầu chạy bản này trên robot thật** (người thứ hai giữ E-Stop; lên mức sau chỉ khi mức trước ổn):
   1. `python main.py --limiter brake` (luật cũ, an toàn hơn nhưng trễ 0.4-0.8 s - chỉ để kiểm tra, không dùng để điều khiển): chờ badge tự lên `ENGAGED`, tay robot **không được nhúc nhích**. Bóp Trigger đưa tay chậm ±5 cm từng trục.
   2. `Ctrl+C` receiver khi đang `ENGAGED`: laptop phải về `SYNC`; chạy lại receiver -> tự `ENGAGED`, tay không nhảy.
   3. `python main.py --speed 4` và receiver `--speed 4` (direct + khóa trục + scale 1:1; giảm scale trên bảng web nếu robot không theo kịp): đưa tay chậm rồi nhanh dần; **dừng tay đột ngột** và quan sát cổ tay/khuỷu có rung hay vọt qua không (luật `direct` không giới hạn gia tốc khi giảm tốc - mới kiểm trên A2SIM, MC giả không mô phỏng việc MC thật từ chối bước nhảy).
   4. Lượt 3 chạy `--speed 4` ở cả hai bên; ổn thì lên `--speed 5`, rồi mới thử mức 6 (rung mạnh hơn). Xem mục "THỬ TỐC ĐỘ CAO LẦN ĐẦU".
   - Dừng ngay nếu thấy `FAULT`, `REJECT`, rung, giật, hoặc tay đứng khựng dù vẫn đang bóp nút; gửi `logs/real_*.jsonl` và `trace_*.jsonl` để phân tích.
5. **Thao tác tay cầm (ánh xạ 1:1 kiểu "nhập vai robot"):**
   - **Trigger** = tịnh tiến (nâng/hạ tay = ±Z, đưa ra/kéo về = ±X, trái/phải = ±Y); **Grip** = xoay cổ tay cùng chiều; cả hai = vừa tịnh tiến vừa xoay.
   - X nhấn = tay trái Home, A nhấn = cả hai tay Home (không cần giữ); cần analog = nắm/mở bàn tay (sau Home vẫn nắm được ngay); tay cầm không còn nút E-STOP.
   - Vươn quá tầm: robot dừng ở biên, rút tay về thì khớp lại đúng chỗ cũ. Gizmo trên web chạy theo tay khi đang bóp nút.
   - Quan sát 2 tay robot **đứng yên hoàn toàn** tại vị trí ban đầu (Anchored pose).
   - Nhấn giữ nút **Trigger** (Cò ngón trỏ) hoặc **Grip** (Cò cạnh bàn tay) để bóp ly hợp (Clutch) và di chuyển: hai cánh tay robot thật sẽ bám theo chuyển động tay của bạn trong không gian!
   - Khi nhả Grip/Trigger: cánh tay robot bám tới vị trí đích cuối cùng, hội tụ êm và khóa đứng yên hoàn toàn (`TARGET_FROZEN`, không bị giật, không hunting).

### Cách B: Điều khiển bằng Chuột / 3D Gizmo trên trình duyệt Laptop
1. Mở trình duyệt trên máy trạm: `https://localhost:8012`
2. Kiểm tra thanh trạng thái: `SYNC` -> `READY (auto-engaging...)` -> `ENGAGED` (tự động).
3. Hệ thống tự Engage sau khi SYNC xong (không còn nút Engage). Bấm nút **Home** / **T-Pose** / kéo Gizmo $\pm X, \pm Y, \pm Z$).
4. Robot sẽ chuyển sang trạng thái `ENGAGED` và bám theo mục tiêu 1:1 mượt mà.
5. Khi nhả chuột (`pointerup`, `pointercancel`, `blur`, `Escape`): Gizmo dừng đúng vị trí nhả chuột, robot bám tới đích và đứng yên tuyệt đối (`TARGET_FROZEN`).
6. Thanh trượt bàn tay (Hand Close) có tác dụng ngay từ đầu trên cả Sim và Robot thật.
7. Nếu di chuyển 2 tay lại quá gần ($< 50$ mm): HUD hiển thị cảnh báo chữ vàng `Arm proximity: XX mm`, robot tự động giảm tốc mềm (không giật dừng, không khóa FAULT) và cho phép di chuyển lùi ra ngoài hoàn toàn tự do.
8. Nếu gặp chướng ngại hoặc chạm biên vật lý gây FAULT: bấm nút **Clear Fault** trên banner để đưa cánh tay về trạng thái an toàn `READY`.

---

## THỬ TỐC ĐỘ CAO LẦN ĐẦU (MỨC 4 → 5 → 6)

Mặc định là mức 6 (từ 17/11). Khi đổi mức tốc độ, lần đầu trên robot thật hãy đi từng mức, có người giữ E-stop, quanh hai tay robot trống 1 m. Hai bên **luôn cùng mức**.

| Lượt | Orin | Laptop | Trần khớp | Theo kịp chuyển động tay 1:1 (đo 13/11) |
|---|---|---|---|---|
| 1 | `python arm_teleop_rx.py --speed 4` | `python main.py --speed 4` | 1.0 rad/s | ~79% thời gian |
| 2 | `python arm_teleop_rx.py --speed 5` | `python main.py --speed 5` | 1.5 rad/s | ~90% |
| 3 | `python arm_teleop_rx.py --speed 6` | `python main.py --speed 6` | 2.0 rad/s | ~95% |

Mỗi lượt (khoảng 2 phút):
1. Receiver in `lvl=<mức>` và `[pacer] ... 400 Hz`; `main.py` in `[SPEED] Active Speed Level: <mức>`.
2. Engage, bấm **Home** một lần: tay về HOME êm, không giật mạnh, không báo lỗi.
3. Kéo chuột chậm một quãng 10-15 cm rồi thả; sau đó dùng tay cầm vung chậm, rồi vung nhanh dần theo cả 3 hướng.
4. Đạt nếu: tay robot bám kịp, dừng không vọt lố, receiver không in `FAULT` / `REJECT` / `DUAL PUBLISHER`, Terminal laptop không báo `Tracking deviation`.
5. Dừng ngay và quay về mức thấp hơn nếu: tay khựng lại giữa đường (MC từ chối bước nhảy), rung mạnh hơn hẳn, có tiếng kêu lạ ở khớp, hoặc robot mất thăng bằng.

Sau mỗi lượt chép trace để đối chiếu: `scp 'agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/trace_orin_*.jsonl' logs/`.

---

## THU DỮ LIỆU (TÙY CHỌN)

- Bắt đầu / dừng một episode: nhấn nhanh **B** hoặc **Y** trên tay cầm, hoặc nút **Start REC** ở thẻ **Data Recording** trên bảng web (điền tên task trước).
- **Discard last** trên bảng web: gắn cờ bỏ cho episode vừa ghi, kèm lý do ở ô **Discard reason** (rơi đồ, va chạm, quỹ đạo xấu, không hoàn thành, lỗi robot, khác). Tay cầm không còn nút discard.
- Điền ô **Instr.** (câu lệnh cho VLA) trước khi ghi. Trong lúc ghi, Orin nén ảnh camera đầu (màu thành JPEG, độ sâu thành PNG 16-bit) và gửi về laptop (cổng TCP 5007); receiver in `[cam] streaming ...`.
- Dữ liệu ghi thẳng trên laptop: `~/a2_ws/recordings/<task>/<YYYY-MM-DD>/ep_<n>_<HHMMSS>/` (trước 17/11 là `data/`) gồm `meta.json`, `robot.jsonl` (khớp: vị trí, vận tốc, mô-men; bàn tay; lệnh; 3 dấu thời gian), `teleop.jsonl` (tay cầm, mục tiêu, pose bàn tay robot) `images/head/*.jpg` và `images/head_depth/*.png`; `robot.jsonl` có thêm khớp cổ và mảng lực/áp lực bàn tay.
- Thẻ web hiện số dòng và số ảnh đã ghi; nếu báo `NO ROBOT DATA` thì receiver chưa chạy hoặc là bản cũ; lỗi camera hiện ngay dưới đó.
- Xuất sang LeRobot v3.0: `venv_lerobot/bin/python tools/export_lerobot.py --task <task> --out lerobot_datasets/<ten> --repo-id local/<ten>` (tự cắt đoạn đứng yên đầu / cuối, giữ 0.5 s; `--no-trim` để tắt). **Từ 17/11 không cần chạy lệnh này**: sau mỗi lần dừng ghi, laptop tự nối episode vào `lerobot_datasets/<task>` (nền, ưu tiên thấp; thẻ Data Recording hiện `LeRobot: added to lerobot_datasets/<task>`; tắt bằng `A2_AUTO_EXPORT=0`). Hướng dẫn đầy đủ về thu data: `DATA_COLLECTION.md` (thư mục gốc dự án). Kiểm đếm data (tập thô, bộ LeRobot, tập nào đã xuất): `venv/bin/python tools/data_status.py`. Xem dataset đã xuất trên dashboard: panel 8012 → nút **Dataset** (timeline, camera, robot 3D, biểu đồ, sự kiện, sửa / cắt / xuất lại; README mục 3.8). Xem episode thô bằng lệnh: `venv/bin/python tools/view_episode.py --task <task>`. Chi tiết: README mục 3.

---

## BƯỚC 5: DỪNG PHIÊN ĐIỀU KHIỂN AN TOÀN

- Khi kết thúc điều khiển: Bấm `Ctrl + C` trên Terminal SSH của Orin. Node sẽ tự động giữ nguyên vị trí đo thực trong 2 giây trước khi nhả lệnh.
- Nếu có bất thường: **Dập ngay nút E-Stop vật lý trên tay người giám sát** hoặc nút E-STOP trên bảng web (tay cầm Quest không còn nút E-STOP).

---

## PHỤ LỤC: BẢNG BISECT DAO ĐỘNG & BIẾN MÔI TRƯỜNG (BISECT MATRIX)

Hệ thống hỗ trợ 6 cờ biến môi trường (mặc định giữ nguyên chuẩn, in rõ lúc khởi động trên cả Laptop và Orin):
- `CART_FOLLOWER=0/1` (Mặc định `1`): Bật/tắt bộ lọc Cartesian critically-damped trước IK (bộ lọc tiền IK duy nhất).
- `WRIST_SCALE=0.5/1.0` (Mặc định `0.5`): Hệ số giảm gia tốc cho 4 khớp cổ tay (Left: J5, J6; Right: J12, J13).
- `WIRE_HZ=50/100` (Mặc định `100`): Tần số phát UDP qua mạng.
- `PLAYOUT_MS=30/50` (Mặc định `30` từ 17/11): Độ trễ đệm khử jitter/drop gói tin trên Orin.

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

### Cơ Chế Tự Va Chạm Giáp Bọc Toàn Diện (Armor Collision Check & Soft Block):
- **Nguyên lý Giáp Bọc (Armor Shell)**: Mọi liên kết va chạm (collision links) trong URDF được thổi phồng thêm 1 cm (`shell_m: 0.010` m trong `config/safety.yaml`). Va chạm được tuyên bố khi hai lớp giáp chạm nhau, tương ứng khoảng cách bề mặt thực $d \le 2 \times \text{shell\_m} = 0.020$ m (20 mm). Khoảng cách bề mặt tối thiểu luôn $\ge 0.019$ m trên mọi đường vào lệnh.
- **Phạm vi kiểm tra toàn diện (1,425 cặp ứng viên)**: Bao phủ 8 nhóm liên kết va chạm:
  1. `arm-arm` (81 cặp giữa 2 cánh tay)
  2. `arm-torso` (18 cặp giữa cánh tay và thân `base_link`)
  3. `arm-head` (36 cặp giữa cánh tay và các đốt đầu)
  4. `arm-pelvis` (108 cặp giữa cánh tay và khung chậu `pelvis`)
  5. `arm-thigh` (126 cặp giữa cánh tay và đùi trên)
  6. `arm-shin` (126 cặp giữa cánh tay và cẳng chân dưới)
  7. `hand-body` (912 cặp giữa bàn tay ngón tay và toàn bộ thân/chân/đầu)
  8. `in-arm non-adjacent` (18 cặp không liền kề trong cùng cánh tay)
- **Đồng bộ hóa hình học kiểm tra**: Dữ liệu kiểm tra va chạm (`scratch_data`) được khởi tạo đồng bộ với tư thế đứng chuẩn (`mujoco.mj_resetDataKeyframe(m, d, 0)`), triệt tiêu sai lệch hình học chân/thân giữa kiểm tra va chạm và hiển thị 3D ($< 0.1$ mm).
- **Bộ lọc bước quét & Fast-Path**: Tại mỗi chu kỳ 100 Hz, thuật toán kiểm tra nhanh điểm đích (endpoint fast-path); nếu tiệm cận biên va chạm mới kích hoạt swept check đa điểm và chia đôi nhị phân (bisection). Chi phí tính toán cực thấp: $\approx 0.25$ ms/tick (dưới ngân sách 1.0 ms).
- **Khóa mềm (Soft Block)**: Khi tiếp xúc biên va chạm, cánh tay giữ nguyên tư thế hợp lệ cuối cùng với $v=0$ (PLAN bit-identical, std = 0, zero sign changes). Không phát sinh FAULT, không cần Clear Fault, không hiện banner đỏ.
- **Thoát tiếp xúc (Contact Retreat)**: Mọi bước di chuyển làm tăng khoảng cách ($d_{\text{cand}} > d_{\text{curr}}$) đều được chấp thuận ngay lập tức, cho phép cánh tay rút ra khỏi vùng tiếp xúc mượt mà không bị kẹt. Trạng thái BLOCKED nhả với ngưỡng trễ hysteresis $\ge 2$ mm.
- **Chặn toàn bộ 8 cổng vào lệnh**: Đảm bảo an toàn không có đường vòng (bypass) qua 8 luồng điều khiển: `gizmo drag`, `nudge buttons +-x/y/z`, `Home / T-pose`, `release cold IK`, `reach recall`, `VR clutch`, `hand tracking`, và `IK fallback`.

### Nút Tinh Chỉnh Descartes (Nudge Buttons +-x/y/z):
- **Cơ chế hoạt động**: Khi bấm các nút $\pm X, \pm Y, \pm Z$ trên Web UI:
  1. Trình duyệt gửi kèm tọa độ Gizmo hiện tại (`cur_pos: [x, y, z]` từ `TransformControls`).
  2. Server cộng dồn bước dịch chuyển (`step_m = 0.02` m) từ tọa độ nhận được, unfreeze trạng thái `TARGET_FROZEN`, và giải IK.
  3. Server phát thanh gói tin `gizmo_set {arm, pos, rot, source: "nudge"}` để đồng bộ vị trí Gizmo 3D trên Web ngay lập tức.
  4. Cánh tay robot bám theo mục tiêu mới và dừng lại với độ lệch chuẩn $\text{std} = 0$ sau 3 giây (không bị chattering 200 Hz do tắt continuous follower).
  5. Cho phép bấm liên tiếp dồn bước (rapid chaining), Gizmo và cánh tay di chuyển mượt mà liên tục.
  6. Tuân thủ 100% cơ chế Armor Soft Block: nếu bước nudge hướng vào thân robot, chuyển động tự động dừng ở biên giáp an toàn $\ge 0.019$ m.

### Phạm Vi Dự Báo Của Môi Trường Mô Phỏng `--sim`:
- **Những gì `--sim` dự báo chính xác (1:1 với Robot thật)**:
  - Biên dạng chuyển động đơn điệu không overshoot (Monotone velocity/acceleration profile).
  - Tự va chạm giáp bọc mềm (Armor collision soft block & contact retreat).
  - Điều khiển tinh chỉnh nút bấm (Nudge buttons chaining & settling std = 0).
  - Phép chiếu Cartesian vươn xa và thu hồi mục tiêu (Reach projection & recall).
  - Chốt dừng khi nhả thao tác (Freeze-on-release & rest deadband hold).
- **Những gì `--sim` KHÔNG dự báo (Cần lưu ý trên Robot thật)**:
  - Ngưỡng từ chối bước nhảy đột ngột của Motion Controller phần cứng (`|step| > 0.02` rad/tick).
  - Độ đàn hồi và ma sát phi tuyến của dây cáp truyền động cánh tay (cable elastodynamics & backlash).
  - Quá tải mô-men xoắn động cơ khi mang tải nặng (payload torque & thermal limits).

### Cơ Chế Xử Lý Mục Tiêu Ngoài Tầm Với & Độ Chính Xác IK (Reach Projection & IK Accuracy):
- **Phép chiếu Cartesian (Cartesian Reach Projection)**: Khi mục tiêu vượt ngoài tầm vươn ($R_{\max} = 0.500$ m tính từ khớp vai URDF `SHOULDER_ORIGIN_LEFT`: `[-0.03779, 0.19100, 1.27210]`, `SHOULDER_ORIGIN_RIGHT`: `[-0.03779, -0.19100, 1.27210]`), hệ thống tự động chiếu mục tiêu về mặt cầu biên $R_{\max}$ dọc theo hướng kéo.
- **Bảo toàn 1:1 trong tầm với**: Khi khoảng cách từ vai $d \le R_{\max}$, phép chiếu trả về chính xác 100% tọa độ mục tiêu (sai lệch $0.0$ mm).
- **Chỉ số sai số IK trên HUD**: HUD đo sai số giữa EE thực tế và mục tiêu *sau khi chiếu vào tầm với* (sai số solver thực sự). Phần vượt tầm với hiển thị riêng bằng cảnh báo vàng: `Target out of reach (X cm beyond)`.
- **Ưu tiên vị trí & Nới lỏng hướng (Position-First Relaxation)**: Trọng số `posture_task` được chuẩn hóa về $0.01$ (nullspace) giúp solver không bị giằng co với mục tiêu Cartesian. Ở biên tầm với hoặc góc khớp hẹp, trọng số hướng tự động nới lỏng ($3.0 \to 0.9 \to 0.15 \to 0.0$) để bảo đảm độ chính xác vị trí luôn $<3$ mm (HUD báo `(ori X°)` khi nới lỏng).
- **Chủ quyền Gizmo & Chốt thả**: Thao tác thả chuột/bóp nhả Grip chốt cứng mục tiêu và hội tụ IK mượt mà; loại bỏ hoàn toàn lỗi kẹt 277 mm.

### Thanh Trượt Điều Khiển Bàn Tay (Hand Slider):
- **Hiệu lực tức thì**: Thanh trượt bàn tay hoạt động ngay lập tức từ khi vừa bật máy (`fresh start`) ở cả `--sim` và `real-udp` mà không cần bấm Open/Close trước.
- **Thứ tự thao tác**: Có thể tự do xen kẽ thanh trượt và các nút Open / Close / Pinch / Home / T-pose theo bất kỳ thứ tự nào.
- **Giới hạn an toàn**: Lệnh bàn tay được mã hóa trong khoảng $[0..2000]$ raw (hết hành trình) với dòng lực an toàn default 1140 (20%), được kiểm tra va chạm ngón tay trước khi xuất lệnh.

### Trạng Thái Orin rx_state & Khởi Động Lại:
- `RX_OK`: Nhận lệnh bình thường và đang xuất điều khiển robot.
- `NOT_ENGAGED`: Laptop chưa SYNC xong để tự Engage / chưa bóp Grip.
- `HOLDING`: Session lock bị giữ bởi phiên cũ hoặc IP lạ. Khắc phục: Dừng gửi 2s để Orin tự nhả lock, sau đó hệ thống tự Engage lại.
- `REJECT:<reason>`: Gói tin bị từ chối do quá cũ (>0.4s) hoặc sai định dạng.
- `FAULT:<reason>`: Bị chốt lỗi an toàn (ví dụ phát hiện node khác gửi đè). Cần restart `arm_teleop_rx.py`.

---

## TÌM GIỚI HẠN TỐI ƯU TỪNG KHỚP (17/11)

Chạy trên Orin, **không chạy receiver**. Mỗi lần chỉ một khớp của tay phải chạy qua lại quanh HOME, ở nhiều tốc độ và gia tốc. Trước khi quét, công cụ tự kiểm tra cả 14 khớp có theo lệnh không.
```bash
python3 joint_limit_sweep.py --dry          # xem kế hoạch
python3 joint_limit_sweep.py                # J1..J7 ở 0.4..2.0 rad/s, gia tốc 6 (~3.5 phút)
python3 joint_limit_sweep.py --joints 1,2,3,4 --speeds 1.2 --accels 3,4,6,8,10
```
Laptop:
```bash
scp 'agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/joint_sweep_*.jsonl' logs/
venv/bin/python tools/joint_sweep_report.py $(ls -t logs/joint_sweep_*.jsonl | head -1) --plot
```
Báo cáo cho độ rung đầu ngón tay (mm) theo khớp / tốc độ / gia tốc và đề xuất `JOINT_VMAX` / `JOINT_AMAX`. Kết quả 17/11 (lúc khuỷu đang hỏng, chỉ tham khảo):
- J1 và J2 rung nhiều nhất, tăng gần tuyến tính theo tốc độ.
- J3 rung khoảng ¼ mức đó, J5 rất ít.
- Gia tốc 3–10 rad/s² không đổi mức rung.

## QUY TRÌNH ĐO RUNG VÀ TRỄ (16/11)

Dùng khi tay robot rung, giật hoặc trễ, để biết lỗi nằm ở lệnh của mình hay ở robot. Chi tiết cách đọc: [docs/FILTER_REPORT.md](docs/FILTER_REPORT.md).

1. **Laptop:** chép bộ ghi sang Orin (đã nằm trong lệnh scp ở BƯỚC 0; bản 16/11 tối không còn mất bản tin)
   ```bash
   scp tools/cmd_recorder.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/
   ```
2. **Orin, terminal thứ hai** (nạp môi trường ROS như BƯỚC 2, mục 4), trong lúc receiver đang chạy:
   ```bash
   cd /agibot/data/home/agi/Desktop/mc
   python cmd_recorder.py      # ghi mọi lệnh tay + trạng thái khớp 100 Hz vào Desktop/logs/trace_recorder_<giờ>.jsonl
   ```
   Teleop 30–60 s (đi chậm, đi vừa, vài lần dừng hẳn), rồi Ctrl+C bộ ghi.
3. **Laptop:** kéo về và phân tích
   ```bash
   scp 'agi@192.168.2.50:/agibot/data/home/agi/Desktop/logs/trace_recorder_*.jsonl' logs/
   scp 'agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/trace_orin_*.jsonl' logs/     # trace của receiver (tuỳ chọn)
   venv/bin/python tools/vibration_report.py $(ls -t logs/trace_recorder_*.jsonl | head -1) --plot
   ```
   - Dòng `COMMAND STREAM` phải báo khoảng 400 msg/s và `0 publisher stalls while moving`. Khoảng hở được chia 3 loại: `publisher stalls` = pacer bị bỏ đói (kiểm tra `sudo prlimit`, BƯỚC 2, mục 6); `recorder losses` = bộ ghi bỏ sót, lệnh vẫn được phát (vô hại); `at rest` = lúc đứng yên, vô hại.
   - Mức tham chiếu (16/11): cảm biến khớp tự có "rung giả" ~4–6% tốc độ (PlanningMove đo được 4,1–5,7%); teleop mức 3 đo được 7,6–8,7%. Rung 5–50 Hz còn lại chủ yếu do robot (đo được gấp 7–13 lần nội dung lệnh).
   - So hai lần chạy (vd mặc định và bản cũ `DIRECT_OUT_FIR=12 DIRECT_OUT_FIR2=10`): `venv/bin/python tools/vibration_report.py logs/trace_recorder_A.jsonl logs/trace_recorder_B.jsonl`.
   - Cột `verdict`: `robot shakes on a smooth command` = rung do robot; `our command is rough` = lệnh còn gồ ghề.
   - Log phía laptop (`logs/trace_laptop_<giờ>.jsonl`, `logs/real_<giờ>.jsonl`) tự ghi mỗi phiên; gửi kèm khi báo lỗi, ghi rõ giờ và thao tác lúc đó.
   - Đồng hồ Orin chạy sớm hơn laptop khoảng 1 giờ, nên tên file hai bên lệch giờ.

