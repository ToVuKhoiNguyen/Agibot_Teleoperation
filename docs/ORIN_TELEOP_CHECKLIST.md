# Bảng Kiểm Tra & Quy Trình Dừng Khẩn Cấp Teleop Trên Orin

> **TRẠNG THÁI KIỂM CHỨNG:** Toàn bộ các bước dưới đây áp dụng cho phần cứng thực tế và được phân loại là **CHƯA ĐƯỢC KIỂM CHỨNG TRÊN PHẦN CỨNG THẬT (UNVERIFIED ON REAL HARDWARE)**.
> Mọi thao tác bắt buộc phải có tối thiểu 02 kỹ sư có mặt tại hiện trường: 01 người vận hành máy tính và 01 người trực tiếp cầm nút E-Stop cơ học.

---

## 1. Bảng Tiêu Chí Dừng Khẩn Cấp & Quy Trình Khôi Phục (Hard STOP & Rollback)

| Sự Cố / Hiện Tượng | Nguy Cơ Tiềm Ẩn | Hành Động Dừng Khẩn Cấp Ngay Lập Tức | Quy Trình Khôi Phục & Đưa Về An Toàn |
| :--- | :--- | :--- | :--- |
| **Độ lệch góc $> 0.05\text{ rad}$ kéo dài $> 200\text{ ms}$** | Va chạm cơ khí, kẹt khớp hoặc mất bước servo | Nhấn `Ctrl-C` trên terminal `arm_teleop_rx.py` (node sẽ tự động phát lệnh giữ vị trí đo thực trong 2 giây). | 1. Ngắt kết nối teleop.<br>2. Kiểm tra log độ lệch khớp trên terminal.<br>3. Kiểm tra ngoại quan xem cánh tay có bị va quẹt hay chạm giới hạn cơ khí không. |
| **Phát hiện `motion_player` đang chạy song song** | Xung đột ghi đè topic 100 Hz gây rung giật cháy động cơ | Dập ngay nút E-Stop vật lý hoặc ngắt tiến trình thu bằng `Ctrl-C`. | 1. Chạy lệnh: `killall -9 motion_player`.<br>2. Xác minh bằng `ps aux \| grep motion_player` (kết quả rỗng).<br>3. Khởi động lại `arm_teleop_rx.py` ở chế độ `--dry`. |
| **Tiếng rít servo tần số cao, rung lắc cơ học** | Vọt lố bộ lọc hoặc cộng hưởng quán tính | Nhấn nút E-Stop cơ học cắt nguồn động lực toàn bộ tay robot. | 1. Hạ robot xuống giá đỡ cơ khí.<br>2. Kiểm tra lại hệ số vận tốc kẹp `--vel-limit` (giảm xuống $0.15\text{ rad/s}$).<br>3. Đánh giá lại độ cứng vững (stiffness/damping) của controller. |
| **Mất kết nối mạng $> 300\text{ ms}$ (Watchdog trip)** | Robot mất kiểm soát khi không còn dữ liệu lái | Không cần thao tác khẩn nếu watchdog hoạt động (node tự động khóa vị trí hiện tại). | 1. Kiểm tra lại cáp mạng Ethernet và switch.<br>2. Kiểm tra bộ đếm `stale` và `seq` trong log thống kê của node.<br>3. Chỉ tiếp tục khi đường truyền ping $\le 2\text{ ms}$. |
| **Robot tự ý đổi trạng thái sang chế độ không mong muốn** | Xung đột từ dịch vụ bên thứ ba hoặc bộ điều khiển RL | Dập nút E-Stop vật lý ngay lập tức. | 1. Đưa robot về trạng thái mặc định an toàn qua giao diện vendor.<br>2. Kiểm tra RPC port 56322 xem có tiến trình nào khác đang gửi lệnh `SetAction` không. |

---

## 2. Bảng Các Yếu Tố Chưa Rõ Cần Làm Rõ Trên Phần Cứng Thật (Unknowns & Assumptions)

1. **Chu kỳ phản hồi của mạch servo cánh tay:** Tài liệu vendor nêu tần số phát khuyến nghị 100 Hz ($10\text{ ms}$), nhưng chưa rõ bộ điều khiển cấp thấp có tự động nội suy spline giữa các chu kỳ hay không.
2. **Hành vi thực tế khi nhận lệnh giữ 2 giây lúc tắt:** Cần đo kiểm thực tế xem động cơ có bị rơi tự do do trọng lực khi node kết thúc hay bộ hãm phanh cơ học (brake) sẽ tự động đóng.
3. **Phản hồi của MC Service khi chuyển action:** Chưa rõ thời gian trễ khi chuyển sang `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO` là bao nhiêu milligiây trước khi chấp nhận dữ liệu từ topic.

---

## 3. Danh Mục Kiểm Tra Vận Hành Từng Bước (Operator Checklist)

*Người vận hành và người giám sát bắt buộc phải ký nháy / đánh dấu xác nhận vào từng ô trước khi tiến hành.*

### Giai Đoạn Chuẩn Bị Cơ Khí & Điện (Pre-flight Inspection)
- [ ] **[ ] BẢO HIỂM TREO:** Robot đã được treo chắc chắn trên cần cẩu/khung gantry bằng cáp chịu lực, hai chân cách mặt đất $\ge 10\text{ cm}$.
- [ ] **[ ] VÙNG AN TOÀN:** Bán kính $2.0\text{ m}$ xung quanh robot hoàn toàn không có người, vật cản, bàn ghế hoặc dây cáp chùng.
- [ ] **[ ] NÚT E-STOP:** Nút dừng khẩn cấp cơ học đã được thử nghiệm ngắt mạch thành công và nằm chắc chắn trên tay kỹ sư giám sát.
- [ ] **[ ] KẾT NỐI MẠNG:** Laptop và Orin kết nối qua cáp mạng Ethernet trực tiếp Cat6, ping thử nghiệm $\le 1.5\text{ ms}$, không rớt gói nào trong 100 lần ping liên tục.
- [ ] **[ ] KHÔNG CHẠY TRÙNG:** Lệnh `ps aux | grep -E "motion_player|arm.py"` trên Orin trả về kết quả 0 tiến trình.

### Giai Đoạn Đo Kiểm Phần Mềm & Khởi Động (Software Gating & Startup)
- [ ] **[ ] KIỂM TRA MÃ:** Chạy `grep -nE "set_action|ensure_action|SetAction" arm_teleop_rx.py` trên Orin, kết quả: **0 dòng**.
- [ ] **[ ] CHẾ ĐỘ THỬ (--dry):** Chạy `python3 arm_teleop_rx.py --dry --allow-ip <IP>`, xác nhận topic `/motion/control/arm_joint_command` có đúng **0 message**.
- [ ] **[ ] TRẠNG THÁI ACTION:** Kiểm tra action hiện tại của robot bằng RPC/giao diện vendor, xác nhận robot đã vào trạng thái `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`.
- [ ] **[ ] BÍ MẬT TOKEN:** Biến môi trường `TELEOP_TOKEN` đã được export đồng nhất trên cả Laptop và Orin:
  - Giá trị token: `________________________________________`
- [ ] **[ ] THÔNG SỐ KHỞI CHẠY:**
  - Địa chỉ IP Laptop cho phép: `________________________`
  - Giới hạn vận tốc đặt (`--vel-limit`): `0.3 rad/s` (mặc định)
  - Giới hạn gia tốc đặt (`--accel-limit`): `2.0 rad/s^2` (mặc định)
- [ ] **[ ] XÁC NHẬN NEO VỊ TRÍ:** Sau khi bật `--live`, terminal hiển thị log: `Anchored to measured pose` và cánh tay robot đứng yên tuyệt đối trong 5 giây đầu tiên.

### Giai Đoạn Thử Nghiệm Từng Khớp & Dừng Hệ Thống (Jogging & Shutdown)
- [ ] **[ ] NHẤP NHẢ KHỚP CỔ TAY:** Gửi lệnh dịch chuyển góc nhỏ $0.01\text{ rad}$ trên khớp `idx19` (cổ tay trái), cánh tay chuyển động êm ái $\le 0.5^\circ$.
- [ ] **[ ] KIỂM TRA MẤT GÓI:** Ngắt kết nối mạng có chủ đích, xác nhận robot tự động giữ nguyên tư thế hiện tại, không rơi hay vung tay.
- [ ] **[ ] KẾT THÚC AN TOÀN:** Nhấn `Ctrl-C` trên terminal Orin, quan sát quá trình giữ tư thế trong 2 giây diễn ra êm đẹp trước khi tiến trình tắt hoàn toàn.

---

**Chữ ký xác nhận người vận hành (Operator):** ___________________________    **Ngày:** ___/___/2026

**Chữ ký xác nhận người giám sát an toàn (Safety Officer):** _________________    **Ngày:** ___/___/2026

