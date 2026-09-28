# ภาพรวมสถาปัตยกรรม RoboMaster EP

เอกสารนี้อ้างอิงโค้ดต้นแบบ commit `ae3c1ab` ของ `YABYABuwu/Template_robomaster_project` ที่นำมาเริ่มโปรเจคนี้ ใช้คู่กับ [README](../README.md) สำหรับวิธีรันและค่าตั้งต้น

## เส้นทางการทำงาน

```text
config/settings.yaml ──> src/config_loader.py ──> main.py
                                             │
                                             ├─ Robot.initialize()
                                             ├─ SensorLogger.start() ── SDK subscriptions
                                             │       ├─ latest + bounded history ใน RAM
                                             │       └─ queue → CSV writer → data/raw/<run>/
                                             ├─ ChassisController.move_to() ← position/attitude จาก logger
                                             │       └─ PID → robot.chassis.drive_speed()
                                             ├─ SlamWorker ← pose + gimbal yaw + selected ToF range
                                             │       ├─ occupancy grid + local scan matching
                                             │       └─ DFSExplorer → GimbalMoveAction(CAR) + ChassisController.move_to()
                                             └─ Dashboard.start() ← logger + robot.camera + SLAM map
                                                     └─ /api/status, /api/history, /video

review.py → RunStore(data/raw) → /api/runs, /api/run, /api/csv → review/index.html
```

`main.py` เป็นเจ้าของวงจรชีวิตของหุ่นยนต์: ตรวจ config ก่อนเชื่อมต่อ, เริ่ม logger, สร้าง chassis controller, เปิด dashboard ถ้าตั้งค่า, รัน waypoint และหยุด `chassis → dashboard → logger → robot` ใน `finally` กรณีหยุดตามเงื่อนไขที่คาดหมายจะบันทึกสถานะ `stopped` และสาเหตุใน `run_summary.json` โดยไม่แสดง traceback; ข้อบกพร่องที่ไม่คาดหมายยังบันทึก `failed` หน้า review รันแยกได้โดยไม่ต่อหุ่นยนต์

## หน้าที่ของแต่ละส่วน

| ส่วน | หน้าที่และจุดเชื่อม |
| --- | --- |
| `config/settings.yaml`, `src/config_loader.py` | ค่าการเชื่อมต่อ, motion/PID, logging, dashboard, review, mission และการตรวจค่าก่อนต่อหุ่นยนต์ |
| `src/logger.py` | ตาราง `STREAMS` ผูกชื่อ stream กับ SDK module/subscribe/unsubscribe/ชื่อคอลัมน์; callback เก็บค่าล่าสุดและ history; writer thread เขียน CSV เฉพาะ stream ที่ `save: true` |
| `src/chassis.py`, `src/PID.py` | `move_to` ใช้ position/attitude ที่ยังสด, แปลงความเร็วจากกรอบโลกเป็นกรอบรถ, จำกัดความเร็วรวมและอัตราเร่งเฉพาะแกนสไลด์ซ้าย/ขวาตาม `motion.max_lateral_accel_m_s2` (ลดความเร็ว/หยุดทันทีได้) แล้วหยุดรถเมื่อจบหรือผิดพลาด; `PIDController` คำนวณค่าควบคุม |
| `src/rear_ir.py` | แปล `adapter` IO เป็น IR หน้า/หลังซ้ายขวาจาก ID/port และ polarity แยกตัว; กั้นคำสั่งเคลื่อนที่เข้าหาวัตถุแล้วหลบเป็นขั้นภายในระยะ/จำนวนครั้งจำกัด โดยตรวจ IR อีกปลายก่อนหลบและรอข้อมูลสดเมื่อขาดช่วง |
| `src/dashboard.py`, `dashboard/index.html` | server ภาพสดและ telemetry; `/api/status` ส่งค่าล่าสุด, `/api/history` ส่งจุดใหม่พร้อมชื่อคอลัมน์, `/video` ส่ง MJPEG |
| `src/slam.py` | `CellWallGrid` เก็บ occupancy ของขอบช่องที่ใช้ร่วมกัน; `OccupancyGridSLAM` ฉายลำแสง ToF เดี่ยว โดยใช้ pose, มุม yaw ของ gimbal และ offset จากแกน yaw; `SlamWorker` เลือก ToF channel ตาม config, อ่าน sample timestamp ใกล้กันและทำงานเบื้องหลัง |
| `src/explorer.py` | `DFSExplorer` หัน gimbal ดูเพื่อนบ้านครบ 4 ทิศเมื่อเข้าช่องใหม่, รอ angle telemetry, action สำเร็จ และ median ของ ToF scan ใหม่ 3 ครั้งที่ตรงทิศ; ก่อนเดินไปช่องใหม่ตรวจ ToF เทียบเกณฑ์กำแพงด้วย scan สดหนึ่งครั้งแล้วใช้ผลนั้นเริ่มเดิน โดยไม่มีการสั่ง scan ทิศเดิมซ้ำ ทางกลับใช้ขอบเปิดที่บันทึกไว้และหัน ToF ตามทิศเดินหนึ่งทิศเพื่อเฝ้าระยะ โดยไม่สแกนสี่ทิศซ้ำ; ก่อนเคลื่อนที่แต่ละช่วงจับ yaw สดแล้วส่งเป็นเป้าหมายคงที่ให้ `ChassisController.move_to()` และหยุดเมื่อ telemetry/status/ทางกลับไม่ผ่านเกณฑ์ |
| `src/targets.py`, `src/target_inspection.py` | ตรวจ HSV สีและรูปร่างจากภาพ BGR ของ dashboard, ติดตาม contour ข้ามภาพ, เล็งด้วย gimbal action ที่รอ SDK ปลด action และส่งคำสั่ง infrared หนึ่งครั้งเมื่อเป้าอยู่ที่จุดเล็งต่อเนื่อง; DFS เรียกหลังสแกน/จัดกลางช่องสำหรับหน้าผนังที่วัดตรง |
| `src/run_review.py`, `review/index.html`, `review.py` | อ่าน run จาก CSV/summary, ส่งข้อมูลตัวอย่างให้กราฟ, ดาวน์โหลด CSV เต็ม, แสดงปัญหาบางประเภทและการเล่นย้อนหลัง |
| `tests/` | ทดสอบ config, PID, logger, controller, dashboard และ review ด้วย fake robot/ข้อมูลชั่วคราว |

## สัญญาข้อมูลที่ใช้ร่วมกัน

- `STREAMS[name]` ระบุชื่อ stream, อุปกรณ์ SDK, เมธอด subscribe/unsubscribe และลำดับคอลัมน์ ลำดับนี้ต้องตรงกับ tuple ที่ callback ส่งมาและแถว CSV
- `logging.streams.<name>` มี `enabled`, `save`, `frequency_hz`; `enabled` เปิด subscription และ history ส่วน `save` เพิ่ม CSV (ต้องเปิด `enabled` ด้วย) position/attitude ต้องเปิดเมื่อ mission ทำงาน
- `SensorLogger.get_latest(name, max_age_s)` คืน tuple หรือ `None` หากยังไม่มี/เก่าเกินกำหนด `get_history_since(after_id)` คืน `{"cursor": id, "streams": {name: [(id, elapsed_s, values), ...]}}` โดย history มีเพดานต่อ stream
- `Dashboard.snapshot()` ส่ง `streams` ของ stream ที่เปิด, `dropped_csv_rows`, `camera_ready`, `camera_error`, `mission_status`, `motion_settings` ที่ใช้จริง; ตัวเลข/อาร์เรย์จาก tuple ถูก JSON แปลงเป็น array และค่าเก่าเกิน 2 วินาทีเป็น `null`
- `Dashboard.history()` เพิ่ม `columns` ให้ข้อมูล history หน้าเว็บใช้ชื่อคอลัมน์สร้างกราฟทั่วไปอัตโนมัติ; API ใช้ `since` เป็น sample id รวมทุก stream
- เมื่อบันทึกข้อมูล: `data/raw/<timestamp>/<stream>.csv` มี `timestamp,elapsed_s,<columns...>`; `run_summary.json` เขียนเมื่อหยุดปกติ และเก็บสถานะ, error, จำนวนแถว, แถวตกหล่น, ค่า stream, `motion_settings` และข้อผิดพลาด logger
- `RunStore.load_run()` ส่ง `streams.<name> = {columns, samples, total_rows, displayed_rows, gap_count, largest_gap_s, first_s, last_s}` โดย `samples` เป็น `[row_number, elapsed_s, values]`; หน้า review สร้างกราฟทั่วไปจาก CSV ที่มีอยู่
- `GET /api/map` ส่ง `robomaster-occupancy-grid` version 1: `resolution_m`, `width`, `height`, `origin`, `data` แบบแถว y เพิ่มจากล่างขึ้นบน (`-1` ไม่รู้จัก, `0` ว่าง, `100` สิ่งกีดขวาง), `pose`, `trajectory`, `counts`, `exploration` และ metadata `sensor_model` (ช่อง ToF, offset/pivot, yaw offset และมุม gimbal ของ scan ล่าสุด)
- Canvas บน dashboard วาด +X ขึ้นเป็นทิศเหนือ, −X ลงเป็นทิศใต้ และ +Y ไปขวาเป็นทิศตะวันออก ทั้งเซลล์แผนที่ เส้นทาง หุ่น และแนว ToF ใช้การฉายพิกัดเดียวกัน การหมุนภาพนี้ไม่เปลี่ยนลำดับข้อมูลใน JSON/ROS หรือคำสั่งเคลื่อนที่
- `GET /api/map/export?format=json` ส่งเอกสาร JSON เดียวกันเป็นไฟล์; `format=png` ส่งภาพ occupancy ขาว/เทา/ดำ; `format=grid-png` ส่งภาพกริด DFS พร้อมขอบกำแพง ทางเปิด และขอบยังไม่รู้; ทั้งสองภาพใช้ +X ขึ้นและ +Y ไปขวา `format=ros` ส่ง ZIP ของ `map.yaml`/`map.pgm`/`slam.json`; `POST /api/map/import` รับ JSON format/version เดียวกันและแทน map ในหน่วยความจำเมื่อ SLAM worker หยุด
- แผนที่บันทึกอัตโนมัติตาม `exploration.map.save_path` พร้อม PNG สองแบบ เช่น `data/maps/latest.json`, `latest.png` และ `latest-grid.png`; ใน `data/raw/<run>/` เก็บ `map.png` และ `map-grid.png` ของแต่ละรัน ทั้งเมื่อจบปกติและหยุดรัน; โหลด JSON ตอนเริ่มด้วย `exploration.map.load_path` ได้เมื่อกรอบพิกัด SDK สอดคล้องกับแผนที่เดิม

## ส่วนที่แสดงผลเฉพาะทาง

กราฟของ stream ตัวเลขใหม่มักขึ้นใน dashboard สดและ review ได้ผ่าน metadata/CSV เดิม แต่ข้อมูลหลักยังมีตำแหน่งเฉพาะใน UI:

| ข้อมูล | Dashboard สด | Review |
| --- | --- | --- |
| position/attitude | แผนที่, ทิศรถ, ความเร็วประมาณ, พิกัด | แผนที่, เวลาเลือก, ความเร็วประมาณ |
| gimbal | มุมบนภาพกล้องและทิศบนแผนที่ | มุมและทิศบนแผนที่ |
| ToF ช่องที่ตั้งใน `exploration.sensor.tof_channel` | ตัวเลขบนภาพกล้องและตำแหน่งหัวเซนเซอร์บนแผนที่ | CSV มีครบสี่ช่องจาก SDK; review แสดงช่อง 0 เป็นค่าเริ่มต้น |
| status | กราฟทั่วไป | เหตุ picked up/slip/impact/roll over |
| camera | MJPEG สดใน RAM | ไม่มีภาพย้อนหลัง |
| เป้าที่ผนัง | ขั้นตอนตรวจผนัง สี/รูปร่าง และจำนวนคำสั่งยิงที่ SDK รับ | ผลตรวจแต่ละหน้าผนังใน run summary; ไม่เก็บภาพย้อนหลัง |
| mission | ข้อความ `mission_status` ใน RAM | สถานะ/error ระดับ run ใน summary |
| exploration | กริดกำแพงสี่ด้าน, occupancy ละเอียด, trajectory, DFS status | กริดสุดท้ายใน summary และตารางกำแพงใน review; JSON export มีทั้งสองกริด |

ดังนั้นการมีกราฟใหม่ไม่ได้ยืนยันว่า dashboard สื่อความหมายของระบบใหม่ครบ โดยเฉพาะสถานะ mission, ความปลอดภัย, หน่วย และเหตุผิดปกติ

## ขอบเขตและข้อควรระวัง

- ระบบเคลื่อนที่ใช้ odometry แบบสัมบูรณ์จาก `sub_position(cs=0)` หน่วยเมตร และ yaw จาก `sub_attitude` หน่วยองศา `drive_speed` ใช้กรอบตัวรถ ไม่มีกลไกหลบสิ่งกีดขวางหรือวางแผนอัตโนมัติ
- ระหว่าง DFS ค่าเริ่มต้น `exploration.heading_source: attitude` ใช้ yaw ของ chassis สำหรับ PID และเล็งทิศสแกน เพราะ log วันที่ 2026-09-27 แสดงว่า yaw ที่อนุมานจาก `gimbal.yaw_ground_deg - gimbal.yaw_deg` ต่างจาก attitude เพิ่มต่อเนื่องประมาณ 0° เป็น 11° ในรัน `20260927_221114_019742`; รัน `20260927_212750_477870` ต่างถึงประมาณ 18° ยังเปิด `heading_source: gimbal` ได้เพื่อทดลองหลังสอบเทียบ โดยปรับศูนย์กับ `sub_attitude` ครั้งแรกและต้องมีข้อมูล gimbal สด ตัวควบคุม yaw ชดเชยต่อเนื่องระหว่างเคลื่อน แม้ความคลาดน้อยกว่าค่า arrival tolerance การเปลี่ยนแหล่ง yaw ไม่ยืนยันว่าหัว ToF ตรงแนวหน้า chassis ทางกล ต้องตรวจบนหุ่นจริง
- `sample_timeout_s` ใช้กับ pose ของ controller; snapshot ของ dashboard ใช้เกณฑ์ 2 วินาที หากเพิ่มข้อมูลที่มีผลต่อคำสั่งขับ ต้องกำหนดเกณฑ์ความสดและพฤติกรรมเมื่อข้อมูลหายเอง
- History ใน RAM มีขอบเขตและอาจสูญจุดเก่าหากผู้ดูดึงช้า CSV ใช้ queue จำกัด; เมื่อเต็มจะข้ามแถวแต่ค่าล่าสุดยังอัปเดต Review อ่านเฉพาะ stream ที่บันทึกไว้
- Dashboard สดเปิดกล้องเมื่อ `dashboard.enabled: true`; หน้า review ไม่มีภาพย้อนหลัง ค่า `host` เริ่มต้นเป็น localhost และเว็บยังไม่มีระบบล็อกอิน
- การทดสอบใน `tests/` ไม่ครอบคลุมหุ่นยนต์จริง การปรับ PID สภาพพื้น และการส่งภาพผ่าน Wi-Fi จริง
- หุ่นยนต์นี้ใช้ ToF เดียวบน gimbal; แต่ละ scan วัดหนึ่งแนวและสะสมเป็น occupancy grid ตาม odometry การ scan matching เดิมต้องมีอย่างน้อยสอง endpoint ใน scan เดียว จึงไม่แก้ drift ในโหมด ToF เดี่ยว และไม่ใช่การ relocalize ทั่วแผนที่
- `exploration.sensor.tof_channel` เป็นดัชนีข้อมูล SDK แบบเริ่มนับจาก 0 (ค่าเริ่มต้น 0); CSV ใหม่ใช้ `tof_0_mm`–`tof_3_mm` ส่วน CSV เก่าที่ใช้ `tof_1_mm`–`tof_4_mm` ยังเปิดใน review ได้ โดยค่าตัวแรกหมายถึงช่อง 0 เหมือนกัน `offset_from_yaw_axis_m` ตั้งเป็น 0.075 m ตามระยะจากแกน yaw ที่ผู้ใช้ให้ และ `offset_yaw_deg: 0` สมมติว่า offset อยู่แนวเดียวกับเลนส์ ส่วน `pivot_x_m/pivot_y_m` ยังตั้งต้นเป็นศูนย์และต้องปรับตามตำแหน่งแกนจริงจากจุดกลางรถก่อนใช้บนฮาร์ดแวร์
- SLAM ไม่กำหนดระยะ ToF สั้นสุด/ไกลสุด; รับค่าบวกที่เป็นตัวเลข finite และข้ามค่า 0 โดยรอ scan ใหม่โดยไม่ยกระดับเป็น error แม้ได้ค่า 0 ซ้ำต่อเนื่อง ค่าระยะใกล้ใน `robot_clearance_m` ยังเป็น scan ที่ใช้ได้ แต่ไม่เพิ่ม occupancy ใต้ตัวรถ และปลายลำแสงที่อยู่นอกกริดจะถูกตัดที่ขอบกริดโดยไม่เพิ่มผนังเทียม Dashboard ระบุเมื่อค่าล่าสุดอยู่ในเขตกันตัวรถ ขณะที่ DFS ใช้เกณฑ์กำแพง 500 มม. จาก ToF ใหม่ก่อนสั่งเดิน
- การอ่าน ToF, gimbal, pose และ status ต้องสดและมี timestamp ใกล้กัน; DFS จะไม่ขับเมื่อมุม gimbal ยังไม่ถึงเป้าหมาย, ไม่มี scan ใหม่ตามทิศ, telemetry ขาด หรือ safety flag ทำงาน
- DFS ใช้ `CellWallGrid` ใน `src/slam.py` เก็บขอบสี่ด้านของช่องและค่า ToF ล่าสุดจากแต่ละทิศ โดยผลวัดตรงไม่ถูกลำแสงไกลเขียนทับ เลือกข้ามเฉพาะขอบ `open` ที่สแกนจากช่องปัจจุบันด้วย ToF มากกว่า `wall_threshold_mm` แล้วสแกนยืนยันทิศนั้นใหม่ แผนที่ละเอียดคงไว้สำหรับดูและ export แต่ไม่ใช้จุด occupancy ระหว่างหมุนหัวมาสร้างกำแพงกริด รายละเอียด schema และการฉายรังสีอยู่ใน [WALL_GRID.md](WALL_GRID.md)
- หลังสแกนสี่ทิศครั้งแรกในช่อง DFS ใช้ผนังที่วัดตรงจัดกลางช่อง โดยชดเชย offset ของหัว ToF ถ้ามีผนังสองฝั่งในแกนเดียวกันจะใช้ครึ่งหนึ่งของผลต่างระยะสองฝั่งเป็นระยะขยับ เพื่อให้สองฝั่งห่างเท่ากัน; ถ้ามีฝั่งเดียวจะใช้ระยะเป้าหมาย 0.20 m (`exploration.alignment.wall_distance_m`) ควบคุมการขยับด้วย PID ของ `ChassisController` โดยจำกัดแต่ละช่วงไม่เกิน `max_shift_m` แล้วสแกนผนังใหม่ก่อนขยับต่อ หากเลยเป้าหมายแต่ระยะคลาดลดลง จะวัดใหม่แล้วขยับกลับในช่วงถัดไป หากระยะคลาดไม่ลดลงให้บันทึก alignment เป็น `stalled` พร้อมระยะที่เหลือ และไม่ขยับจัดกลางต่อในช่องนั้น เกณฑ์หยุดฉุกเฉิน 20 ซม. ไม่ใช้ใน alignment
- หลังขยับจัดกลางช่อง DFS ใช้ scan ผนังที่เพิ่งได้คำนวณรอบถัดไปโดยตรง; หาก scan เกิน `max_sample_age_s` ระหว่างรอ pose/telemetry จะสแกนผนังใหม่ก่อนขยับ
- ระหว่างเดินตามกริดทั้งขาไปและขากลับ DFS หัน ToF ไปตามทิศเดินและตรวจมุม/อายุข้อมูลก่อนใช้ `stop_if` ของ `ChassisController`; เมื่อระยะกึ่งกลาง chassis ถึงกำแพง ≤ `exploration.emergency_stop_distance_m` (0.20 m) controller หยุดทันทีและบันทึก `exploration.last_motion_stop`. เกณฑ์ครึ่งทางเลือกช่องที่รถน่าจะอยู่ แต่ pose หยุดจริงไม่ถูกเขียนทับเป้ากลางช่องโดยอัตโนมัติ. ยืนยันกลางช่องได้จาก pose ใกล้เป้าภายใน tolerance; ระยะผนังด้านเดียวใช้เป็นข้อมูลประกอบแต่ยืนยันกลางช่องไม่ได้; ถ้ายืนยันไม่ได้เก็บ `center_pending` และหยุด DFS ก่อนก้าวถัดไป. ขอบที่พบผนังยังบันทึกด้วยระยะฉายจากเป้ากลางช่อง ส่วนการเดินกลับหันเพียงทิศเดินเพื่อเฝ้าระยะ ไม่สแกนสี่ทิศซ้ำ Dashboard/review แสดงช่องที่นับว่าอยู่ เป้ากลางช่อง ระยะคลาด และ pose จริง
- ตั้ง `exploration.alignment.enabled: false` เพื่อปิดเฉพาะการขยับจัดกลางช่อง DFS ยังสแกนกำแพง ตัดสินทางเดิน และเดินกลับตามปกติ dashboard/review แสดงว่าปิด alignment
- SDK Python 3.8 ที่ติดตั้งกำหนด `GimbalMoveAction` ของ `moveto()` เป็น `COORDINATE_YCPN` (`yaw CAR`) ดังนั้น yaw คำสั่งเทียบ chassis เช่นเดียวกับช่อง `yaw_deg` ใน `sub_angle` ส่วน `yaw_ground_deg` เป็นอีกกรอบที่ห้ามนำมาบวกเข้ากับคำสั่ง DFS เลือกมุมสมมูลในช่วง SDK `[-250°,250°]` ที่ใกล้ relative yaw ปัจจุบันสุด และตรวจมุม scan ในกรอบเดียวกันก่อนอนุญาตให้เดิน
- เมื่อ `auto_recenter` เปิด DFS จะสั่ง `gimbal.recenter()` หนึ่งครั้งก่อนเริ่ม รอ SDK action สำเร็จและ telemetry หลังคำสั่งที่อยู่ใน tolerance; dashboard แสดง `centering_gimbal` ระหว่างขั้นตอนนี้. จากนั้น DFS ใช้ action `COORDINATE_CAR` yaw=0 เมื่อสแกนตรงหน้า และ yaw เทียบ chassis สำหรับทิศอื่น รอ yaw/pitch telemetry และ ToF scan ใหม่ที่ตรงทิศ **และ** SDK action สำเร็จก่อนตรวจทางเดินหรือออกคำสั่ง gimbal ถัดไป ไม่มีเวลาหมดอายุสำหรับการรอข้อมูลเริ่มต้น, action, scan หลังเดิน หรือก้าว DFS
- DFS ใช้ `GimbalMoveAction` แบบ `COORDINATE_CAR` ผ่าน `src/gimbal_control.py` เพื่อสั่ง yaw และ pitch เทียบ chassis ทั้งคู่ แล้วตรวจ `yaw_deg` กับ `pitch_deg` ในกรอบเดียวกัน แทน `moveto()` ที่ล็อก pitch กับกรอบอ้างอิงพื้น
- `exploration.gimbal.angle_tolerance_deg` ใช้ตรวจ yaw ส่วน `pitch_tolerance_deg` ใช้ตรวจ pitch แยกกัน; log วันที่ 2026-09-28 พบ pitch telemetry เปลี่ยนตาม yaw ตั้งแต่ `-6.7°` ถึง `7.1°` แม้ action สำเร็จ จึงใช้ pitch tolerance `8°` จากค่าที่สอบเทียบจริงและคง yaw `3°`. ไม่ใช้ `gimbal.move()` ชดเชย pitch แบบ relative เพราะทำให้ค่าจาก `+7.1°` กลับขั้วเป็น `-6.7°`; หาก SDK action จบแล้วแต่มุมเกิน tolerance และ telemetry สดนิ่งต่อเนื่อง 3 sample จะลอง `moveto` เดิมอีกหนึ่งครั้ง แล้วหยุดพร้อมมุมเป้าหมาย/จริงหากยังไม่เข้าเป้า

อ้างอิง API ตำแหน่ง/ทิศ/ระยะจาก [RoboMaster Python SDK guide](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/robomaster.html) และสเปกช่วงระยะ/FOV ของ ToF จาก [คู่มือ RoboMaster EP](https://dl.djicdn.com/downloads/ROBOMASTER_EP/20220429UM/RoboMaster_EP_User_Manual_v1.2_EN_1.pdf)
