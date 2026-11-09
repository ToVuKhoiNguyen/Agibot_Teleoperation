# Quy Trình Thang Đo Độ Trễ Robot Thật (A/B/C/D Bisect)

Tài liệu hướng dẫn kỹ sư vận hành đo kiểm độ trễ và độ bám động học giữa laptop teleop và robot AGIBOT A2 Ultra thực tế.

## 1. Điều Kiện Tiên Quyết & An Toàn Bắt Buộc
- **Quy tắc 2 người (Two-Person Rule)**: Các bậc Rung C1 trở lên **BẮT BUỘC** phải có 2 người:
  - **Người 1 (Operator)**: Thao tác lệnh trên laptop.
  - **Người 2 (Safety Spotter)**: Đứng cạnh robot, cầm sẵn **E-Stop không dây (Wireless E-Stop)** trên tay.
- Robot đã được bật nguồn, servo bật (Enable), không có bất kỳ cản trở vật lý nào trong bán kính sải tay.
- Domain ROS 2 trên Orin là 232; laptop chạy UDP backend chuyển tiếp qua cổng 5005 (TX) và 5006 (RX).
- **Tuyệt đối không tăng trần vận tốc** hoặc nới lỏng giới hạn an toàn 0.01 rad/cycle.

## 2. Điều Kiện Dừng Khẩn Cấp (STOP Conditions)
Bấm nút **E-Stop** vật lý hoặc nhấn `Ctrl-C` trên terminal ngay lập tức nếu xuất hiện bất kỳ dấu hiệu nào sau đây:
1. **Lỗi hệ thống (FAULT)**: Xuất hiện cảnh báo FAULT hoặc mất gói tin phản hồi trên cổng 5006.
2. **Sai lệch vị trí (Tracking Deviation > 0.05 rad)**: Robot không bám kịp quỹ đạo hoặc lệch tư thế HOME > 0.05 rad.
3. **Tiếng ồn / Rung chấn bất thường**: Động cơ phát tiếng rít, giật cục hoặc rung lắc cơ học ngoài ý muốn.
4. **Nguy cơ va chạm**: Bất kỳ bộ phận nào của cánh tay tiến sát thân robot (< 30 mm) hoặc có tiếp xúc cơ học.

## 3. Thứ Tự Đo Kiểm Chuẩn (Run Order)
Chạy script trên laptop (thay `<ROBOT_IP>` bằng IP thực tế của Orin robot, ví dụ `192.168.2.50`):
```bash
python scripts/real_latency_ladder.py --host <ROBOT_IP> --port 5005 --rx-port 5006
```

Thực hiện tuần tự theo từng nấc thang (mỗi nấc chạy 20 giây):
1. **Đưa robot về HOME**: Xác nhận robot đang ở tư thế chuẩn `[0, 1.2, 0, -0.5, 1.5, 0, 0, 0, -1.2, 0, 0.5, 1.5, 0, 0]`.
2. **Nấc B (Vendor Arm Replay - Tam giác 0.10 rad/s)**:
   - Gõ `RUNG B` và nhấn Enter.
   - Quỹ đạo tam giác $\pm 0.2$ rad tại khớp khuỷu (J4-L và J4-R) với bước 0.002 rad/20ms.
3. **Nấc C1 (Sine $\pm 0.2$ rad, Vận tốc đỉnh 0.20 rad/s)**:
   - Xác nhận Spotter đã sẵn sàng cầm E-Stop.
   - Gõ `RUNG C1` và nhấn Enter.
4. **Nấc C2 (Sine $\pm 0.2$ rad, Vận tốc đỉnh 0.30 rad/s)**:
   - Gõ `RUNG C2` và nhấn Enter.
5. **Nấc C3 (Sine $\pm 0.2$ rad, Vận tốc đỉnh 0.45 rad/s)**:
   - Gõ `RUNG C3` và nhấn Enter (nấc tiệm cận trần an toàn 0.50 rad/s).
6. **Nấc D (Chuyển động người dùng - Quest Teleop)**:
   - Dữ liệu thu thập tự động trong các phiên teleop Quest thực tế lưu tại `logs/real_<timestamp>.jsonl`.
   - Phân tích ngoại tuyến nấc D bằng lệnh:
     ```bash
     python scripts/analyze_ladder.py logs/real_<timestamp>.jsonl
     ```

## 4. Dữ Liệu Cần Copy & Dán Lại Cho Tôi
Sau khi hoàn tất các nấc thang đo:
1. Sao chép toàn bộ khối bảng kết quả in ra màn hình terminal (bao gồm các cột: Rung, Khớp, Trễ, Gain, RMS, Bão hòa, Jitter, Tuổi FB).
2. Tên tệp log đã sinh ra trong thư mục `logs/` (ví dụ `logs/ladder_YYYYMMDD_HHMMSS.json`).
3. Nếu phân tích phiên Quest thực tế (Nấc D), dán thêm bảng phân bố vận tốc khớp và % thời gian tiệm cận ngưỡng (>= 90% cap) từ script `analyze_ladder.py`.

