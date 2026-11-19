# Mô tả dữ liệu robot A2: đọc là hiểu

Tài liệu này giải thích **dữ liệu thu từ robot AGIBOT A2 Ultra là gì, trông thế nào, từng con số nghĩa là gì**. Bạn không cần biết về robot hay học máy. Cách thu và các thao tác nằm ở `DATA_COLLECTION.md`.

---

## 1. Dữ liệu này dùng để làm gì

Người vận hành đeo kính VR và điều khiển hai tay robot làm một việc, ví dụ nhặt cốc đặt lên khay. Máy ghi lại **mọi thứ xảy ra** trong lúc đó. Sau này, một mô hình AI xem lại hàng trăm lần như vậy để học cách tự làm việc đó.

Cách mô hình học rất đơn giản, giống như học lái xe bằng cách xem người lái:

> Nhìn thấy cảnh này (ảnh camera), robot đang ở tư thế này (góc khớp), và được dặn việc này (câu lệnh) → thì người vận hành đã ra **lệnh** này.

Vì vậy dữ liệu chia thành hai nửa:

| Nửa | Tên trong dữ liệu | Nghĩa |
|---|---|---|
| Đầu vào | `observation` (quan sát) | Robot đang **thế nào**: ảnh nhìn thấy, góc các khớp, vị trí tay, lực... |
| Đầu ra | `action` (hành động) | Robot được **ra lệnh gì** ở lúc đó: góc khớp mục tiêu, độ nắm của ngón tay |

---

## 2. Hai dạng dữ liệu: bản thô và bộ train

```
Robot + tay cầm VR
        |
        v   (nhấn B bắt đầu / kết thúc, máy ghi)
  recordings/            BẢN THÔ: ghi đầy đủ, nhiều file, mỗi tập một thư mục
        |
        v   (tự động sau mỗi tập, hoặc xuất tay)
  lerobot_datasets/      BỘ TRAIN: gọn, đồng đều 30 khung/giây, đúng chuẩn LeRobot
```

- **Bản thô** là bản gốc, giữ mọi chi tiết, dùng để kiểm tra và sửa. **Không bao giờ xoá nó**: có bản thô là xuất lại bộ train được, mất bản thô là mất hẳn.
- **Bộ train** là bản đã sắp xếp lại cho mô hình đọc. Nó theo chuẩn **LeRobot v3.0**, một định dạng phổ biến để chia sẻ dữ liệu robot.

Một **episode** (tập) là một lần làm trọn một việc, từ lúc bấm B bắt đầu tới lúc bấm B kết thúc, thường 10 đến 60 giây. Một bộ train gồm nhiều episode của cùng một **task** (nhóm việc).

---

## 3. Bản thô: máy ghi những gì

Mỗi episode là một thư mục: `recordings/<task>/<ngày>/ep_<số>_<giờ>/`. Bên trong:

| File | Ai ghi | Bao nhiêu lần mỗi giây | Chứa gì |
|---|---|---|---|
| `robot.jsonl` | Máy tính trong robot (Orin) gửi về | khoảng 100 | Góc 14 khớp tay đo được, vận tốc, mô-men, **lệnh Orin vừa gửi cho robot**, 12 ngón tay, cổ |
| `teleop.jsonl` | Laptop | tối đa 100 | Tư thế tay cầm VR, mục tiêu của bàn tay, mức nắm, trạng thái đồng bộ, các cảnh báo an toàn |
| `images/head/*.jpg` | Camera đầu robot | tối đa 15 | Ảnh màu 1280×720 |
| `images/head_depth/*.png` | Camera đầu robot | tối đa 15 | Ảnh độ sâu 1280×720 (mỗi điểm ảnh là khoảng cách, đơn vị mm) |
| `images.jsonl` | | mỗi ảnh một dòng | Thời điểm chụp và thông tin của từng ảnh |
| `meta.json` | | | Task, câu lệnh, ngày giờ, thời lượng, có bị đánh dấu bỏ không, điểm cắt |

Đuôi `.jsonl` là file chữ, mỗi dòng là một mẫu đo, mở được bằng trình soạn thảo nào cũng xem được.

---

## 4. Bộ train: bên trong có gì

```
lerobot_datasets/<task>/
  data/chunk-000/file-000.parquet      BẢNG SỐ: mỗi dòng là một khung hình
  videos/observation.images.head/...   VIDEO ảnh màu (.mp4)
  videos/observation.images.head_depth/...   VIDEO ảnh độ sâu (.mp4)
  meta/
    info.json          mô tả cấu trúc: số khung, fps, tên và kích thước từng trường
    stats.json         nhỏ nhất, lớn nhất, trung bình, độ lệch của từng trường
    tasks.parquet      các câu lệnh
    episodes/*.parquet mỗi episode bắt đầu và kết thúc ở khung nào
    a2_sources.json    (riêng dự án này) mỗi episode lấy từ tập thô nào
```

**Parquet** là một dạng file bảng số, giống Excel nhưng gọn và đọc rất nhanh. Bạn không cần mở trực tiếp, đọc bằng Python ở mục 8.

**Bộ là cả thư mục.** Thiếu `data/`, `videos/` hoặc `meta/` là không dùng được.

### Một "khung hình" là gì

Dữ liệu được lấy mẫu đều **30 lần mỗi giây**. Mỗi lần là một **khung** (frame), giống một tấm ảnh chụp nhanh toàn bộ tình trạng robot tại một thời điểm. Một episode dài 25 giây có khoảng 750 khung. Mỗi dòng của bảng parquet là một khung.

Ví dụ thật: bộ `default` hiện có 2 episode, 1973 khung, tức khoảng 66 giây.

### Các cột (trường) của mỗi khung

**Đầu vào (observation):**

| Trường | Số con số | Nghĩa |
|---|---|---|
| `observation.state` | 62 | Trạng thái tổng hợp, chia ba nhóm ở bảng dưới |
| `observation.images.head` | ảnh 720×1280×3 | Ảnh màu camera đầu (3 kênh đỏ, xanh lá, xanh dương) |
| `observation.images.head_depth` | ảnh 720×1280×1 | Ảnh độ sâu camera đầu (1 kênh, mm) |
| `observation.ee_pose` | 14 | Vị trí và hướng thật của hai bàn tay robot |
| `observation.velocity` | 14 | Vận tốc từng khớp tay (rad/s) |
| `observation.effort` | 14 | Mô-men (lực xoay) từng khớp tay |
| `observation.neck` | 2 | Cổ robot: xoay ngang, gật |
| `observation.hand_effort_raw` | 12 | Lực ở các ngón tay (số thô) |

**Đầu ra (action):**

| Trường | Số con số | Nghĩa |
|---|---|---|
| **`action`** | 26 | **Lệnh thật gửi cho robot**: 14 góc khớp tay + 12 ngón tay. Đây là thứ mô hình học dự đoán |
| `action.ik_target` | 14 | Mục tiêu khớp thô tính trên laptop, là ý định của người vận hành trước khi qua các bộ giới hạn. Chỉ để tham khảo |
| `action.ee_pose` | 16 | Lệnh theo dạng bàn tay: vị trí và hướng hai tay, cộng mức nắm trái và phải (0 đến 1) |

**Do LeRobot tự thêm:** `timestamp` (giây từ đầu episode), `frame_index` (số thứ tự khung trong episode), `episode_index` (episode số mấy), `index` (số thứ tự khung trong cả bộ), `task_index` (tra câu lệnh trong `tasks.parquet`).

### Cách đọc 62 số của `observation.state`

| Vị trí | Số con số | Nghĩa |
|---|---|---|
| 0 – 6 | 7 | Góc 7 khớp **tay trái** (`left_arm_joint1` đến `7`), đơn vị rad |
| 7 – 13 | 7 | Góc 7 khớp **tay phải** (`right_arm_joint1` đến `7`), đơn vị rad |
| 14 – 19 | 6 | 6 ngón **tay trái**: `thumb_0`, `thumb_1` (hai khớp ngón cái), `index` (trỏ), `middle` (giữa), `ring` (áp út), `pinky` (út). Số thô, khoảng 0 đến 2000 |
| 20 – 25 | 6 | 6 ngón **tay phải**, cùng thứ tự |
| 26 – 34 | 9 | **Bàn tay trái robot** thật: vị trí x, y, z (m) + 6 số mô tả hướng |
| 35 – 43 | 9 | **Bàn tay phải robot** thật, cùng dạng |
| 44 – 52 | 9 | **Tay cầm VR trái**: vị trí + hướng |
| 53 – 61 | 9 | **Tay cầm VR phải**: vị trí + hướng |

Nếu xuất với tuỳ chọn `--no-controller-pose` thì bỏ hai nhóm tay cầm, còn **44** số. Nên làm vậy nếu lúc chạy mô hình thật sẽ không có người cầm tay cầm.

Và `action` có 26 số: 14 góc khớp (cùng thứ tự với vị trí 0 – 13 ở trên) rồi 12 ngón tay (cùng thứ tự với vị trí 14 – 25).

---

## 5. Đơn vị và các khái niệm khó

- **Khớp (joint) và góc (rad).** Mỗi tay robot có 7 khớp, giống vai, khuỷu, cổ tay của người. Góc đo bằng **radian** (rad): 1 rad khoảng 57 độ, một vòng tròn là khoảng 6,28 rad.
- **Vị trí (x, y, z).** Đơn vị mét. Vị trí bàn tay robot tính so với thân robot. Vị trí tay cầm VR tính so với kính VR, hai hệ này khác nhau.
- **Hướng của bàn tay.** Có hai cách ghi, đều mô tả cùng một việc là bàn tay đang xoay hướng nào:
  - **Quaternion** (`qx, qy, qz, qw`, 4 số) trong `ee_pose`.
  - **6 số hướng** (`r00, r10, r20, r01, r11, r21`, là hai cột đầu của ma trận xoay) trong `observation.state`. Cách này liên tục, ít gây rắc rối khi huấn luyện hơn quaternion.
- **Ngón tay (số thô 0 đến 2000).** Mỗi ngón là một số. Số lớn nhỏ tương ứng ngón duỗi hay co, chưa quy ra độ. Mức nắm `closure` trong `action.ee_pose` là số gộp từ 0 (mở) đến 1 (nắm chặt).
- **Mô-men (effort).** Lực xoay mà động cơ khớp đang dùng. Khớp càng gánh nặng thì số càng lớn.
- **IK (động học ngược).** Phép tính từ "bàn tay muốn tới chỗ này" ra "các khớp phải xoay bao nhiêu". Nếu chỗ đó quá xa tầm với, IK không tới đích được.

---

## 6. Thời gian trong dữ liệu

Các nguồn ghi ở tốc độ khác nhau (robot 100 lần/giây, camera 15 lần/giây). Khi xuất, mọi thứ được đưa về **một nhịp chung 30 khung/giây**. Với mỗi khung, máy lấy **mẫu mới nhất có tại hoặc ngay trước thời điểm đó**. Hệ quả:

- Camera 15 ảnh/giây nên **mỗi ảnh được lặp lại ở 2 khung liên tiếp**. Đây là bình thường.
- Mọi trường trong cùng một khung được căn theo cùng một thời điểm.

Hai thao tác tự động làm cho episode gọn hơn:

- **Cắt đứng yên đầu và cuối.** Đoạn đầu và cuối mà tay robot không cử động bị bỏ, giữ lại 0,5 giây ở mỗi đầu. Episode mà robot không hề cử động thì bị bỏ hẳn.
- **Cắt do người vận hành.** Trên trang Dataset có thể cắt thêm, kể cả các đoạn đứng yên ở giữa. Khi đó hai phần còn lại nối liền nhau, nên tại chỗ nối robot có thể nhảy vị trí trong một khung.

---

## 7. Ảnh và video

- **Ảnh màu** (`head`): camera đầu, 1280×720, lưu thành video `.mp4`. Mã hoá là AV1, nên một số trình xem mặc định không mở được (xem ở dưới).
- **Ảnh độ sâu** (`head_depth`): mỗi điểm ảnh là khoảng cách tới vật, đơn vị mm. Lưu thành video 12 bit xám, nén chất lượng rất cao. **Không dùng để xem bằng mắt**: mở bằng trình phát thường sẽ thấy tối hoặc nhiễu. Muốn xem thì dùng trang Dataset, nó tô màu theo khoảng cách (xanh là gần, đỏ là xa, chỗ đen là không đo được).
- **Kích thước.** Cả hai đều 1280×720 từ ngày 17/11. Dữ liệu ghi trước ngày đó là 640×360 và bị phóng lên 1280×720 khi xuất, nên vẫn mờ.

**Mở video bị lỗi "AV1 decoder is required"?** Bộ giải mã AV1 chưa có trên máy. Mở bằng `mpv` hoặc `vlc`, hoặc xem trên trang Dataset.

---

## 8. Đọc dữ liệu bằng Python

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset("local/pick_cup", root="/home/<bạn>/a2_ws/lerobot_datasets/pick_cup")
print(ds.num_episodes, ds.num_frames, ds.fps)        # số episode, số khung, 30

frame = ds[0]                                        # khung đầu tiên
print(frame["observation.state"].shape)              # (62,)
print(frame["action"].shape)                         # (26,)
print(frame["observation.images.head"].shape)        # (3, 720, 1280): ảnh màu, giá trị 0 đến 1
print(frame["task"])                                 # câu lệnh, ví dụ "pick up the cup ..."

left_arm = frame["observation.state"][0:7]           # 7 góc khớp tay trái (rad)
```

Cài thư viện: `uv pip install "lerobot[dataset-viz]==0.7.0"` (đã có trong `venv_lerobot`).

---

## 9. Những điều dễ nhầm

| Nhầm lẫn | Sự thật |
|---|---|
| "action là cái người vận hành bấm" | `action` là lệnh **thật sự gửi cho robot sau khi đã qua các bộ giới hạn tốc độ**. Robot bám theo nó trong khoảng 60 ms. Ý định thô của người vận hành nằm ở `action.ik_target` |
| "Ảnh bị trùng khung là lỗi" | Camera 15 ảnh/s còn dữ liệu 30 khung/s nên mỗi ảnh lặp 2 lần |
| "Mở video màu mà báo thiếu bộ giải mã" | Dùng `mpv` hoặc `vlc` |
| "Video độ sâu thấy tối thui" | Đó là dữ liệu số, không phải ảnh để nhìn. Xem trên trang Dataset |
| "Bộ chỉ cần file parquet" | Phải có đủ cả `data/`, `videos/` và `meta/` |
| "Sao `observation.state` có 62 số mà mô hình thật chỉ có 44" | 18 số cuối là tay cầm VR, chỉ có khi có người cầm. Dùng `--no-controller-pose` nếu không muốn mô hình phụ thuộc vào chúng |
| "Người nhận chỉ có bộ train, sao không sửa hay cắt được" | Sửa, cắt và xem cảnh báo an toàn cần bản thô trong `recordings/` |

---

## 10. Bảng từ ngữ

| Từ | Nghĩa |
|---|---|
| Episode | Một lần làm trọn một việc |
| Task | Tên nhóm việc, cũng là tên bộ dữ liệu |
| Instruction | Câu mô tả việc bằng ngôn ngữ, mô hình dùng để biết phải làm gì |
| Frame (khung) | Một lần chụp nhanh toàn bộ trạng thái, 30 lần mỗi giây |
| Observation | Robot đang thế nào (đầu vào của mô hình) |
| Action | Lệnh đưa cho robot (đầu ra mô hình phải đoán) |
| Joint (khớp) | Chỗ xoay của tay robot, đo bằng góc (rad) |
| End effector (ee) | Bàn tay của robot |
| IK | Tính từ vị trí bàn tay ra góc các khớp |
| Orin | Máy tính nằm trong robot, nhận lệnh và gửi dữ liệu về |
| ENGAGED | Trạng thái robot đang bám theo tay cầm của người vận hành |
| Parquet | Định dạng file bảng số gọn và nhanh |
| LeRobot | Thư viện và chuẩn dữ liệu robot dùng cho bộ train |
