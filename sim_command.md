# Hướng Dẫn Chạy Mô Phỏng MuJoCo Y Hệt Robot Thật (A2SIM)

Mục tiêu: gõ **đúng nguyên văn** các lệnh trong `real_robot_command.md`, nhưng thay vì robot thật, laptop kết nối tới
robot A2 mô phỏng bằng MuJoCo. Test và xác nhận trên sim trước, sau đó mới chạy robot thật.

```
Laptop (192.168.2.75)                         "Robot" = network namespace a2sim (192.168.2.50, không có đường ra ngoài)
  python main.py  ── UDP 5005/5006 ──▶  arm_teleop_rx.py (file thật, scp lên như robot)  ◀── ssh agi@192.168.2.50 (mật khẩu 1)
                                              │ ROS 2 domain 232  /motion/control/*
                                              ▼
                                         Robot giả: MuJoCo (thân, tay, ngón, va chạm, giới hạn khớp, servo ±100 Nm)
                                           + đáp ứng MC đo từ robot thật (config/mc_plant.json)
                                           + RPC 192.168.100.100 (SetAction/GetAction/motion_player)
```

**Giống robot thật:** mọi lệnh, IP, cổng, user/mật khẩu, đường dẫn `/agibot/...`, ROS domain 232, file receiver, SetAction.py
và disable_motion_player.sh của hãng, mọi kiểm tra an toàn và trạng thái (action 402, motion_player, PUB_CHECK, PREFLIGHT,
DUAL PUBLISHER, HOLDING, FAULT), tư thế đầu HOME, tay va chạm thân thì bị chặn vật lý.
**Chưa giống (xem cuối file):** độ trễ/quán tính tay là tạm thời (UNVALIDATED), bàn tay lý tưởng, thân đứng yên tuyệt đối.

---

## CHUẨN BỊ MỘT LẦN (cần mật khẩu sudo)

> **Ngắt kết nối mạng robot thật trước** (sim dùng lại IP 192.168.2.75/.50 – script sẽ từ chối nếu thấy mạng robot).
> Tắt các phiên cũ nếu đang chạy: `Ctrl+C` các `main.py`, rồi `pkill -f sil/fake_mc_node.py; pkill -f arm_teleop_rx.py`.

```bash
cd ~/a2_ws
sudo -E bash sil/a2sim/setup.sh
bash sil/a2sim/status.sh          # kỳ vọng các dòng OK (robot chưa bật là bình thường)
```
Script này: cài `openssh-server` (dịch vụ ssh hệ thống bị TẮT, laptop không mở cổng 22 ra mạng công ty), tạo user `agi`
(mật khẩu `1`) với thư mục `/agibot/data/home/agi/Desktop/{mc,motion_player}`, tạo mạng ảo 192.168.2.75 ↔ 192.168.2.50,
chạy sshd chỉ trong mạng ảo, thêm khối `a2sim` vào `~/.ssh/config` để khóa host của robot giả không lẫn với robot thật.
Sau khi khởi động lại máy, chạy lại lệnh setup (mạng ảo mất khi reboot).

---

## 0. Hướng Dẫn Nhanh Chạy Mô Phỏng Gắp Vật Vào Thùng (--sim)

Chạy mô phỏng MuJoCo độc lập trên Laptop (kèm giao diện Web 3D và kịch bản gắp lon Pringles, cuộn băng dính, bóng trắng vào thùng carton):

```bash
cd ~/a2_ws
source venv/bin/activate
python main.py --sim
```

- **Giao diện Web điều khiển**: Mở trình duyệt vào `https://localhost:8012` (hoặc mở Quest Browser vào địa chỉ này).
- **Cửa sổ 3D MuJoCo**: Tự động mở hiển thị robot, bàn, thùng carton và 3 vật thể.
  - **Chuyển đổi góc nhìn (Multi-view Cameras)**: Bấm phím `[` hoặc `]` trên bàn phím (hoặc chọn menu **Camera** trên thanh công cụ MuJoCo) để chuyển đổi tức thì giữa các góc nhìn:
    - `cam_side`: Nhìn ngang tầm bàn (quan sát tay và mặt bàn).
    - `cam_top`: Nhìn vuông góc từ trên trần xuống bàn và thùng carton.
    - `cam_front`: Nhìn trực diện robot và bàn thao tác.
    - `cam_operator`: Góc nhìn từ phía sau người vận hành.
    - `head_front_cam`: Góc nhìn thứ nhất từ đầu robot nhìn xuống.
- **Thao tác gắp**:
  - Di chuyển tay robot tới vật thể bằng chuột (kéo Gizmo trên Web) hoặc bằng tay cầm Quest VR.
  - Khi ngón tay nắm lại ($\ge 45\%$), vật thể tự động dính vào bàn tay để nhấc lên.
  - Đưa vật qua miệng thùng carton và mở bàn tay ($\le 30\%$), vật thể sẽ rơi tự do vào thùng.
  - Bấm nút **Reset** trên giao diện Web bất cứ lúc nào để đưa robot về tư thế Arms Up và xếp lại các vật về vị trí ban đầu.

---

## 0b. Thông Số A2SIM (Mô phỏng cả khối Orin giả định)
- **Máy trạm Teleop (Laptop):** `192.168.2.75`
- **Máy tính nhúng Robot (Orin):** `192.168.2.50` *(SSH user: `agi`, mật khẩu: `1`)*
- **Địa chỉ giao diện Web:** `https://192.168.2.75:8012`
- **UDP lệnh / feedback:** `5005` / `5006`

---

## BẬT NGUỒN ROBOT (thay cho bấm nút nguồn)

**Terminal R:**
```bash
cd ~/a2_ws
bash sil/a2sim/robot_power_on.sh          # hỏi mật khẩu sudo; cửa sổ MuJoCo = "robot thật"
# A2SIM_VIEWER=0 bash sil/a2sim/robot_power_on.sh   # không mở cửa sổ MuJoCo
```
Kỳ vọng: `[fake_mc] action=McAction_RL_LOCOMOTION_DEFAULT motion_player=ON` – robot vừa bật: **chưa** ở 402, motion_player **đang** chạy.

> **Chỉ bật nguồn MỘT LẦN, như robot thật (robot cứ để bật).** Terminal R giữ nguyên giữa các lần thử. Mỗi lần chạy lại chỉ cần
> Terminal 1 (`main.py`) và Terminal 2 (`arm_teleop_rx.py`) – y như robot thật, không cần `pkill` hay bật nguồn lại.
> - Nếu bạn lỡ đóng cửa sổ MuJoCo: nó **tự mở lại** khi một phiên teleop mới bắt đầu (lệnh tay đầu tiên sau ≥ 3 s im lặng,
>   tức là khi `arm_teleop_rx.py` + `main.py` chạy lại).
> - Chỉ cần bật nguồn lại (`pkill -f sil/fake_mc_node.py` rồi `robot_power_on.sh`) khi muốn đưa robot về trạng thái "vừa bật"
>   (action chưa ở 402, motion_player ON, tay về HOME), hoặc sau khi sửa code trong `sil/`.
> - Sau khi robot đã ở 402 từ phiên trước, bước `SetAction.py` (10 rồi 12) và `disable_motion_player.sh` ở BƯỚC 2 vẫn chạy được lại như thường.

---

## BƯỚC 0: ĐỒNG BỘ CODE SANG ROBOT (y nguyên)

```bash
# Kiểm tra mã băm SHA256 trước khi gửi:
sha256sum scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py
# Kì vọng arm_teleop_rx.py:   2c1b2ab0ef9736a9e87710da981cf86b0f314be14625240192032fb1fea7e212
# Kì vọng orin_teleop_core.py: 45b9a4c6ff8bd2f3599db526becb9a77af37bcaf83be1b18a27fdfaa19d3e3b9  (17/11 – BẮT BUỘC chép lại CẢ HAI nếu Orin còn bản cũ)

# Đồng bộ sang Orin (cả 4 tệp độc lập vào thư mục mc):
scp scripts/arm_teleop_rx.py scripts/orin_teleop_core.py scripts/orin_trace_logger.py scripts/async_writer.py agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/
```
Lần đầu ssh/scp sẽ hỏi xác nhận khóa host (lưu vào `~/.ssh/known_hosts_a2sim`, không đụng khóa của robot thật).

---

## BƯỚC 1: KHỞI ĐỘNG TELEOP SERVER TRÊN LAPTOP (y nguyên)

**Terminal 1:**
```bash
cd ~/a2_ws
source venv/bin/activate
python main.py                                   # mặc định: speed 6, smooth 1, --limiter direct, khóa trục, scale 1.0 (1:1), --elbow-out 15, --thumb-pose 400,900,800,1500
# python main.py --speed 3 --smooth 3            # chậm hơn, êm hơn (receiver cũng --speed 3)
# python main.py --limiter brake --no-axis-lock  # quay về hành vi cũ
```
| Tùy chọn | Ý nghĩa |
|---|---|
| `--speed 1..6` | Trần tốc độ/gia tốc khớp: 1 = 0.4 rad/s, 2 = 0.6, 3 = 0.8, 4 = 1.0, 5 = 1.5 (4.0 rad/s²), 6 = 2.0 rad/s (6.28 rad/s²); mặc định 6 từ 17/11. **Receiver phải chạy cùng số.** |
| `--limiter direct` (mặc định) / `brake` | Bộ giới hạn bám gần như trực tiếp (trễ ~35–100 ms thay vì 100–450 ms), các khớp chậm lại cùng tỉ lệ nên tay đi **thẳng theo trục** (nâng tay = +Z, đưa ra = +X…) thay vì vẽ đường cong; không vọt lố. Đổi lại không giới hạn gia tốc khi giảm tốc. Chỉ đặt ở laptop |
| `--axis-lock` (mặc định) / `--no-axis-lock` | Tay cầm, khóa trục mềm: đưa tay thật thẳng theo một trục (lệch < 12°, nhả khi lệch > 20°) thì robot đi theo trục đó, phần lệch trục vẫn đi 30 %; tay chậm quá 0,25 s hoặc đưa chéo thì tự do 3D. Độ chặt cũ: `AXLOCK_SNAP_DEG=20 AXLOCK_RELEASE_DEG=30 AXLOCK_LEAK=0 AXLOCK_SLOW_FREE_S=0` |
| `--smooth 1` (mặc định) / `2` / `3` / `0` | Làm mượt lệnh của luật `direct`: gia tốc tỉ lệ với sai lệch vận tốc (không còn bật qua lại giữa hai cực), trần gia tốc, và lọc trung bình trượt ở đầu ra. 1 = nhẹ (gia tốc 6.0, 0.04 s, lọc 4 tick), 2 = vừa (4.0, 0.06 s, 8 + 10 tick), 3 = êm nhất (3.0, 0.10 s, 6 + 4 tick, trễ hơn), 0 = tắt (dừng gắt như bản cũ) |
| `--elbow-out 15` (mặc định) / `9` / `0` | Ưu tiên khuỷu tay mở ra ngoài: kéo khuỷu ra khi nó ở gần thân hơn chừng này cm tính từ vai. 9 = không đụng tới tư thế kiểu HOME, 0 = tắt |
| `--hand-raw-max 2000`, `--thumb-pose 400,900,800,1500`, `--thumb-rest 500` | Trần hành trình ngón tay (2000 = nắm hết); ngón cái nghỉ ở tư thế cầm vật (xoay 400 / gập 900) và đóng theo cần gạt tới xoay 800 / gập 1500 khi nắm hết; `--thumb-pose off` = ngón cái theo tay như cũ, khi đó `--thumb-rest` là tư thế nghỉ; receiver có cùng các tham số |
| Biến môi trường `NUDGE_GLIDE=0`, `SOFT_ORI=0`, `XR_SOFT_ORI=0`, `IK_SETTLE_S=0`, `XR_COAST=0`, `HAND_HOLD_MOVING=0`, `DIRECT_OUT_FIR2=1`, `HOME_MAX_VEL=2.0` | Tắt lần lượt: nút dịch trượt mượt, cổ tay thẳng khi kéo chuột, cổ tay thẳng khi tịnh tiến bằng tay cầm, tư thế tự hoàn tất sau khi dừng, đi tiếp theo đường thẳng sau khi nhả trigger, giữ ngón tay khi tay chạy, phanh tăng dần (chỉ bật sẵn ở `--smooth 2/3`), Home đi chậm. Bảng đầy đủ: README mục 2.3 B |
| `--limiter brake` | Luật giới hạn cũ: phanh êm nhưng trễ nhiều (0.4–0.8 s trên robot thật). Muốn ít trễ thì KHÔNG thêm cờ này |
| `--xr-scale 0.6` | Tỉ lệ tay cầm → robot lúc khởi động (mặc định 1.0 = 1:1; 0.6: tay 10 cm → robot 6 cm). Đổi trực tiếp khi đang chạy bằng thanh **Controller → Robot Scale** trên bảng web |
| `--sim` (tự động nạp kịch bản gắp vật vào thùng) | Chạy mô phỏng thuần MuJoCo (không cần robot thật hay A2SIM namespace). **Từ bản cập nhật mới, chỉ cần gõ `--sim` là hệ thống tự động kích hoạt kịch bản gắp vật vào thùng carton**: gồm bàn trắng cao 0.78 m, thùng carton ở giữa, và 3 vật thể thực tế bên ngoài (lon Pringles đỏ, cuộn băng dính đen, quả bóng trắng) trong tầm với tay phải; gắp hỗ trợ: vật dính vào bàn tay nắm ≥ 45 % quanh nó, thả khi dưới 30 %. Thấy trực tiếp trên giao diện web 8012, kính Quest và cửa sổ MuJoCo |

> Nếu trình duyệt báo lỗi tới `https://192.168.2.75:8012`: `export no_proxy="localhost,127.0.0.1,192.168.2.75,$no_proxy"` rồi chạy lại
> (proxy công ty; trên robot thật cũng cần dòng này nếu proxy bật).

---

## BƯỚC 2: KHỞI ĐỘNG ROBOT RECEIVER TRÊN ORIN (y nguyên)

**Terminal 2:**
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
# Kỳ vọng đúng mã: 2c1b2ab0ef9736a9e87710da981cf86b0f314be14625240192032fb1fea7e212

# 4. Nạp cấu hình ROS 2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml

# 5. CHUYỂN ACTION THỦ CÔNG (BẮT BUỘC TRƯỚC KHI BẬT LIVE)
python SetAction.py          # hiện menu: nhập 10 (McAction_STAND_DEFAULT), chờ robot đứng vững (~1 s)
python SetAction.py          # chạy lại: nhập 12 (McAction_STAND_ARM_EXT_JOINT_SERVO)
# Sau bước này robot ở chế độ servo tay ngoài (402); receiver kiểm tra GetAction và từ chối nếu chưa đúng.
#   Y hệt robot thật. Robot giả báo 402 sau khi chọn 12; chọn 19 trực tiếp cũng được. (SetAction.py của hãng bỏ qua --action.)

# 6. Khởi động receiver (robot thật còn bước `sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40` trước dòng này;
#    trong sim bỏ qua được, khi đó pacer in WARNING "not real-time")
python arm_teleop_rx.py                 # cùng mức với main.py (mặc định 6)
# python arm_teleop_rx.py --speed 3     # nếu main.py chạy --speed 3
# python arm_teleop_rx.py --dry
```
> Sau mỗi lần **bật nguồn lại** robot giả (Terminal R), motion_player bật lại: phải chạy lại `disable_motion_player.sh` và SetAction,
> nếu không receiver báo FATAL ở PREFLIGHT và thoát (laptop sẽ báo `stale feedback`).
Kỳ vọng:
```
[arm_teleop_rx] SHA256: 2c1b2ab0ef9736a9e87710da981cf86b0f314be14625240192032fb1fea7e212... | 100Hz | playout=30ms | lvl=6      # lvl phải trùng --speed của main.py
[FEEDBACK] First joint state received! Streaming feedback to 192.168.2.75:5006
[PREFLIGHT] arm: 0.0 msg/s, hand: 0.0 msg/s
[arm_teleop_rx] Ready and listening on 192.168.2.75:5005...
```
Web chuyển `SYNC` → `READY (auto-engaging...)` → `ENGAGED` **tự động** (không còn nút Engage).
Ngay khi vào `ENGAGED`, tay robot giả giữ nguyên vị trí đo (không giật); mọi thao tác chuột/kính từ đây gửi lệnh thật xuống MC.
(Gizmo kéo trong lúc còn `SYNC` bị bỏ qua: khi đồng bộ, mục tiêu được đặt lại về vị trí đo thực.)

### Diễn tập làm sai (phản ứng giống robot thật)
| Làm sai | Kết quả |
|---|---|
| Bỏ bước 5 (SetAction 10 → 12) | `FATAL: Robot action is 'McAction_RL_LOCOMOTION_DEFAULT'. Must set to 402!` |
| Bỏ `disable_motion_player.sh` | `PREFLIGHT ... hand: 10.0 msg/s` → `FATAL: Active pub ... motion_player` *(giả định hành vi motion_player)* |
| Quên BƯỚC 0 / scp bản cũ | `sha256sum` khác `2c1b2ab0ef9736a9e87710da981cf86b0f314be14625240192032fb1fea7e212...` / `45b9a4c6ff8bd2f3599db526becb9a77af37bcaf83be1b18a27fdfaa19d3e3b9...`; bản rất cũ phát 100 hoặc 200 Hz |
| `--speed` khác nhau giữa laptop và receiver | Không lỗi, nhưng receiver kẹp tốc độ theo mức thấp hơn → không thấy nhanh hơn |
| Tắt receiver khi đang ENGAGED | Laptop báo `Robot feedback lost` → `SYNC`; bật lại receiver → tự đồng bộ về vị trí đo rồi `ENGAGED`, tay **không nhảy** |
| Chạy receiver lần 2 khi lần 1 còn chạy | `PREFLIGHT arm: 100 msg/s` → FATAL (node khác đang publish) |

---

## BƯỚC 3, 4, 5: làm đúng như `real_robot_command.md`
- Kiểm tra auto-Engage: badge lên `ENGAGED`, `python sil/sil_ctl.py status` thấy MC nhận lệnh, vị trí không nhảy.
- Thử bàn tay (≤ 2000 raw, ngón cái nghỉ 400 / 900, gập theo khi nắm), nudge, kéo gizmo, thả chuột, Home/T-pose.
- **Kéo gizmo bằng chuột:** tay robot đi theo **ngay trong lúc kéo** (kể cả khi kính Quest đang kết nối), không đợi thả chuột.
- **Tay cầm Quest** (vào `https://localhost:8012` trong kính qua ADB) – ánh xạ 1:1 kiểu "nhập vai robot":

  | Thao tác | Kỳ vọng |
  |---|---|
  | Giữ **Trigger**, nâng/hạ tay | Tay robot đi ±Z (lên/xuống), đúng quãng đường |
  | Giữ Trigger, đưa tay ra trước/kéo về | ±X (ra phía trước **của robot**) |
  | Giữ Trigger, đưa sang trái/phải | ±Y |
  | Giữ **Grip**, chúc/xoay/lật cổ tay | Bàn tay robot xoay cùng chiều (cổ tay dừng ở giới hạn khớp ~25–30°) |
  | Giữ cả hai | Vừa tịnh tiến vừa xoay |
  | Vươn quá tầm rồi rút về | Robot dừng ở biên, rút về thì khớp lại đúng chỗ cũ |
  | Nhả nút | Robot dừng tại chỗ, gizmo về đúng bàn tay |
  | Vung nhanh rồi dừng đột ngột | Không vọt qua, không rung (quan trọng nhất với `--limiter direct`) |

  Gizmo trên panel/kính phải **chạy theo tay** khi đang giữ nút. Vung quá nhanh (> ~0.3 m/s) robot vẫn đi đúng trục nhưng trễ và
  đi ngắn hơn – đó là trần tốc độ khớp, tăng `--speed` để cải thiện. Từ HOME tay robot gần duỗi hết nên hạ xuống/vươn ra rất ít.
- Sau khi cập nhật code: **tải lại trang** (cả trên PC và trong kính) để nhận JS mới.
- Clear Fault: hệ thống về `READY` rồi **tự Engage lại** tại vị trí đo hiện tại (không trườn về đích cũ).
- Muốn tạm ngừng điều khiển (không có nút) – auto-engage bị tắt cho tới khi gọi lại `engage_arm`:
  ```bash
  curl -sk --noproxy '*' -X POST https://127.0.0.1:8012/api/command -H 'Content-Type: application/json' -d '{"name":"disengage_arm","arm":"both"}'
  curl -sk --noproxy '*' -X POST https://127.0.0.1:8012/api/command -H 'Content-Type: application/json' -d '{"name":"engage_arm","arm":"both"}'
  ```
- Quan sát cửa sổ MuJoCo: tay robot giả di chuyển theo lệnh thật gửi xuống MC; nếu tay bị đưa vào thân (bỏ qua armor),
  tay **bị chặn vật lý** và servo đẩy tới 100 Nm – trên robot thật đó là va chạm thật.
- Dừng: `Ctrl+C` receiver (Terminal 2), `exit`; `Ctrl+C` laptop. **Không cần** tắt Terminal R: robot cứ để bật; `Ctrl+C` Terminal R
  chỉ khi hết buổi test (hoặc muốn "tắt nguồn" robot giả).
- **Không có E-Stop vật lý trong sim** – hãy vẫn tập thói quen người thứ hai sẵn sàng như với robot thật.

### Theo dõi / tiêm sự cố (Terminal 3, trong `~/a2_ws`)
```bash
python sil/sil_ctl.py status          # action, motion_player, lệnh MC nhận, vị trí đo, mô-men servo, tiếp xúc va chạm
python sil/sil_ctl.py freeze on       # robot ngừng gửi feedback (tắt: freeze off)
python sil/sil_ctl.py dual on         # node khác publish đè -> FAULT DUAL PUBLISHER
python sil/sil_ctl.py push 3 0.05     # ngoại lực đẩy khớp 3 (0.5 s)
python sil/sil_ctl.py burst 3         # feedback tới theo cụm
```

### Thu trace (y nguyên quy trình thật)
```bash
scp "agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/logs/trace_*.jsonl" logs/
python scripts/classify_arrival.py logs/trace_laptop_<ts>.jsonl logs/trace_orin_<ts>.jsonl logs/trace_recorder_<ts>.jsonl
```
Robot giả ghi thêm `logs/sil/<ts>_a2sim/fake_mc_*.jsonl` (lệnh MC nhận, vị trí đo, lệnh/đo bàn tay).
> Laptop vẫn ghi `logs/real_<ts>.jsonl` khi chạy sim – **đừng** dùng các file này để fit mô hình.

---

## TRƯỚC KHI CHẠY ROBOT THẬT (BẮT BUỘC)

```bash
sudo bash sil/a2sim/teardown.sh             # gỡ mạng ảo 192.168.2.75/.50, dừng robot giả + sshd, gỡ khối ~/.ssh/config
# sudo bash sil/a2sim/teardown.sh --purge   # xóa luôn user agi, /agibot, /etc/a2sim
```
Kỳ vọng dòng cuối: `no 192.168.2.x left on this laptop - ready for the real robot`. Sau đó kết nối mạng robot và làm
`real_robot_command.md` với đúng các lệnh vừa tập.

---

## KHÁC BIỆT CÒN LẠI SO VỚI ROBOT THẬT

| | Ghi chú |
|---|---|
| Bật nguồn | `bash sil/a2sim/robot_power_on.sh` thay cho nút nguồn, chạy một lần rồi để robot bật; cửa sổ MuJoCo tự mở lại mỗi phiên teleop mới |
| Sàn / chân | Sàn MuJoCo đặt ở z = −0.057 để bàn chân chạm đất (thân cố định ở z = 0.93) |
| Armor (thân) | Cặp va chạm với thân dùng ngưỡng riêng `armor_collision.torso_threshold_m` = 5 mm trong `config/safety.yaml` (các cặp khác 20 mm); log `[ARMOR] BLOCKED ...` hiện khoảng cách đã bù |
| Kính Quest | Không với tới IP ảo `192.168.2.75`: dùng ADB reverse (`https://localhost:8012`) hoặc IP thật của laptop |
| Động lực học tay | MuJoCo + đáp ứng MC fit từ 9 s dữ liệu (tay phải mượn tay trái) – UNVALIDATED. Fit lại sau khi chạy `scripts/plant_excitation.py` trên robot thật |
| Bàn tay | Lý tưởng (tới ngay vị trí lệnh), chưa có dữ liệu đo; ngón tay vẫn va chạm vật lý |
| Thân/chân | Đứng yên tuyệt đối (robot thật đang tự cân bằng RL, có lắc nhẹ) |
| MC | Giữ lệnh cuối khi mất lệnh, bám mọi bước nhảy, chỉ nhận lệnh ở action 402 – **giả định**, chưa đo |
| `aima em load-env`, DDS profile Orin | Giả (không có bản của hãng ngoài robot) |
| Mạng | Ảo, gần như không trễ/mất gói (Wi-Fi/LAN thật có trễ) |
| `SetAction.py` (10 rồi 12) | 12 → robot giả báo 402 là suy ra từ cách dùng robot thật (bạn xác nhận 10 → 12), chưa đo GetAction thật. Bản của hãng trong repo hiện menu; nếu Orin có bản riêng, scp bản đó vào `~/Desktop/mc/` của agi |

Chế độ nhanh không sudo (loopback, cho test tự động): `docs/SIL.md`, `venv/bin/python scripts/test_sil.py --e2e`.
