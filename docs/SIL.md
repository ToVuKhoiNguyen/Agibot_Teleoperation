# SIL – Kiểm thử Software-In-the-Loop giống robot thật

Mục tiêu: chạy **đúng đường code của robot thật** trên laptop, không cần robot. Mọi thứ là code thật,
trừ phần MC + động cơ được thay bằng mô hình (plant) đã nhận dạng từ log thật.

```
Laptop: main.py (backend real-udp, KHÔNG --sim)  ROBOT_IP=127.0.0.1
  IK -> SafetyLayer -> MirrorLimiter -> UdpBackend         <- code thật
        │ UDP 5005 (proto v3)            ▲ UDP 5006 feedback
        ▼                                │
Fake Orin: scripts/arm_teleop_rx.py NGUYÊN BẢN (đúng SHA)  <- code thật, ROS 2 Humble, Python 3.10
  drain/session lock/playout 30 ms/final-line guard/hand gate/DUAL PUBLISHER
        │ /motion/control/arm_joint_command  ▲ /arm_joint_state, /hand_joint_state (100 Hz)
        ▼                                    │
sil/fake_mc_node.py                                          <- MÔ HÌNH
  plant MC-only (config/mc_plant.json) + mock RPC GetAction + tiêm lỗi
```

| Thành phần | Trong SIL | Giống thật? |
|---|---|---|
| main.py, IK, SafetyLayer, MirrorLimiter, UdpBackend | chạy nguyên bản | 100 % (cùng code) |
| Giao thức UDP, cổng, single-sender lock | thật, qua loopback | 100 %; mạng: dùng `SIL_NETEM` để giả trễ/mất gói |
| arm_teleop_rx.py + orin_teleop_core/trace_logger/async_writer | file nguyên bản, SHA in ra khi khởi động | 100 % (khác duy nhất: RPC GetAction trỏ về mock 127.0.0.1) |
| ROS 2 topics, QoS BEST_EFFORT, tên khớp, frame `AgiHand` | thật (domain cô lập, localhost) | 100 % (DDS profile của Orin không dùng) |
| MC + động cơ tay | plant trễ + bậc 1/2 + bão hòa | **mô hình**, độ chính xác = chất lượng log nhận dạng |
| Bàn tay | IDEAL cho tới khi có log hand | **mô hình** |

## Chạy

```bash
# Terminal 1 – robot giả (ROS_DOMAIN_ID=88, localhost only; từ chối domain 232 của robot thật)
SPEED=3 bash sil/sil_up.sh

# Terminal 2 – laptop y như khi chạy robot thật, chỉ trỏ về loopback
ROBOT_IP=127.0.0.1 venv/bin/python main.py --speed 3
```
Biến môi trường: `SIL_PLANT=<json>`, `SIL_IDEAL=1` (plant pass-through), `SIL_NETEM="delay 3ms 1ms loss 0.5%"`
(cần sudo), `SIL_RX_ARGS="--no-thumb-swing"`, `SIL_MC_ARGS="--action 10"`.

Tiêm lỗi khi đang chạy: `venv/bin/python sil/sil_ctl.py status | freeze on | dual on | action 10 | burst 3 | push 3 0.05`.

## A2SIM – lệnh y hệt 100% + robot vật lý MuJoCo (khuyến nghị để tập trước khi lên robot)

Xem `sim_command.md`. Tóm tắt: `sudo -E bash sil/a2sim/setup.sh` dựng network namespace `a2sim` (laptop 192.168.2.75 ↔
robot 192.168.2.50, alias RPC 192.168.100.100, không default route), user `agi`/`1`, sshd chỉ trong namespace,
cây `/agibot/...`. `bash sil/a2sim/robot_power_on.sh` bật robot giả (`fake_mc_node.py --a2sim --physics mujoco
--boot real`, ROS domain 232, cửa sổ MuJoCo). Sau đó gõ nguyên văn `real_robot_command.md`. Gỡ trước khi dùng robot
thật: `sudo bash sil/a2sim/teardown.sh`.

Vật lý (`sil/mujoco_plant.py`): lệnh MC → đáp ứng MC đã nhận dạng → servo khớp MuJoCo (kp 3000, kv tới hạn, ±100 Nm,
bù trọng lực) → va chạm/giới hạn/quán tính → giá trị đo + offset/lượng tử hóa cảm biến. Các cặp mesh chồng nhau sẵn ở
HOME (cổ tay link05/07, vai–thân, đầu–thân, chân–sàn) bị loại khỏi va chạm; mọi cặp khác giữ nguyên.

## Chạy theo đúng từng bước của real_robot_command.md (loopback, không sudo)

| Robot thật | SIL |
|---|---|
| Bật nguồn robot | `bash sil/robot_on.sh` (action `McAction_RL_LOCOMOTION_DEFAULT`, motion_player bật) |
| BƯỚC 0 `scp ... agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/` | `cp scripts/{arm_teleop_rx,orin_teleop_core,orin_trace_logger,async_writer}.py sil/orin_fs/agibot/data/home/agi/Desktop/mc/` |
| BƯỚC 1 `python main.py` | `ROBOT_IP=127.0.0.1 venv/bin/python main.py` |
| `ssh agi@192.168.2.50` | `bash sil/ssh_orin.sh` (prompt `[SIM] agi@orin`, `~` = `/agibot/data/home/agi`) |
| `aima em load-env`, `disable_motion_player.sh`, `source ROS`, `export ...`, `SetAction.py` | gõ y nguyên (script của hãng chạy nguyên bản; export ROS nguy hiểm bị chặn) |
| `python arm_teleop_rx.py` | `python arm_teleop_rx.py --allow-ip 127.0.0.1` (khác biệt duy nhất) |

Đường dẫn tuyệt đối `/agibot/...` dùng được nếu tạo symlink (tùy chọn): `sudo ln -s ~/a2_ws/sil/orin_fs/agibot /agibot`.
`SetAction.py` trong SIL là bản của hãng (`agibot_a2_aimdk-dev2.0/examples/mc`): hiện menu, bỏ qua `--action`.
Nếu bản trên Orin khác (hỗ trợ `--action 402`), chép nó vào `sil/orin_fs/agibot/data/home/agi/Desktop/mc/`.

## Kiểm thử tự động

```bash
venv/bin/python scripts/test_sil.py            # Layer A: receiver (A1..A11)
venv/bin/python scripts/test_sil.py --e2e      # + Layer B: main.py -> fake Orin -> fake MC
venv/bin/python scripts/test_sil.py --only B --ideal   # tách hiệu ứng do plant
venv/bin/python tests/test_sim_fidelity.py   # plant in-process (main.py --sim, SIM_PLANT=1)
```
`XFAIL` = lỗi đã biết, đã báo cáo, chưa sửa (không làm suite đỏ). `FAIL` = lỗi chưa xử lý.

## Hai phạm vi plant

| File | Phạm vi | Dùng cho | Nguồn log |
|---|---|---|---|
| `config/mc_plant.json` | Orin publish → Orin đo (chỉ MC) | SIL fake MC | `trace_orin_*.jsonl` |
| `config/sim_plant.json` | laptop gửi → laptop nhận (gộp mạng+Orin+MC) | `main.py --sim` với `SIM_PLANT=1` | `logs/real_*.jsonl` |

Plant tích phân ở bước gốc lúc nhận dạng (`metadata.dt_s`, 10 ms) bất kể tốc độ gọi (200 Hz control loop vẫn đúng thời gian trễ).

## Thu dữ liệu nhận dạng trên robot thật

1. Diễn tập trên SIL: `SPEED=1 bash sil/sil_up.sh` rồi `venv/bin/python scripts/plant_excitation.py --speed 1`.
2. Robot thật (người thứ hai giữ E-stop, receiver `--speed 1`, **không** chạy main.py):
   `venv/bin/python scripts/plant_excitation.py --robot-ip 192.168.2.50 --speed 1 --i-have-estop`
   (kiểm tra offline giới hạn khớp + khoảng hở armor ≥ 40 mm cho toàn bộ quỹ đạo trước khi gửi; tự hủy khi FAULT/stale/lệch lớn).
3. Lấy trace về: `scp agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/logs/trace_orin_<ts>.jsonl logs/`
4. Nhận dạng (chạy 2 lần, 1 file fit + 1 file validate):
   `venv/bin/python tools/identify_plant.py --source orin --fit logs/trace_orin_A.jsonl --val logs/trace_orin_B.jsonl --hand-logs logs/plant_excitation_A.jsonl`

Pipeline nhận dạng đã được kiểm chứng vòng kín trên SIL: dữ liệu sinh từ plant đã biết (J0: trễ 10 ms, tau 72.7 ms;
J5: fn 3.90 Hz, ζ 1.10) được nhận dạng lại thành 10 ms / 75.0 ms và 3.65 Hz / 1.06 (sai số ~3 %, do Orin lấy mẫu state bất đồng bộ).

## Giả định chưa đo (gắn nhãn ASSUMED trong config)

- MC giữ lệnh cuối khi ngừng nhận `arm_joint_command` (receiver luôn publish hold nên chỉ quan trọng khi receiver chết).
- MC bám mọi bước nhảy (không mô hình từ chối bước) – `mc_step_reject.enabled=false`.
- MC bỏ qua lệnh ngoài action servo; state 100 Hz; đơn vị tay raw 0..2000.
- Plant tuyến tính từng khớp: không trọng lực/tải/ghép khớp, không va chạm môi trường.
