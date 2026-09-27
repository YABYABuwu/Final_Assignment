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
                                             │       └─ DFSExplorer → gimbal.moveto() + ChassisController.move_to()
                                             └─ Dashboard.start() ← logger + robot.camera + SLAM map
                                                     └─ /api/status, /api/history, /video

review.py → RunStore(data/raw) → /api/runs, /api/run, /api/csv → review/index.html
```

`main.py` เป็นเจ้าของวงจรชีวิตของหุ่นยนต์: ตรวจ config ก่อนเชื่อมต่อ, เริ่ม logger, สร้าง chassis controller, เปิด dashboard ถ้าตั้งค่า, รัน waypoint และหยุด `chassis → dashboard → logger → robot` ใน `finally` ความล้มเหลวของ mission ถูกบันทึกใน `run_summary.json` ก่อนส่ง exception ต่อ หน้า review รันแยกได้โดยไม่ต่อหุ่นยนต์

## หน้าที่ของแต่ละส่วน

| ส่วน | หน้าที่และจุดเชื่อม |
| --- | --- |
| `config/settings.yaml`, `src/config_loader.py` | ค่าการเชื่อมต่อ, motion/PID, logging, dashboard, review, mission และการตรวจค่าก่อนต่อหุ่นยนต์ |
| `src/logger.py` | ตาราง `STREAMS` ผูกชื่อ stream กับ SDK module/subscribe/unsubscribe/ชื่อคอลัมน์; callback เก็บค่าล่าสุดและ history; writer thread เขียน CSV เฉพาะ stream ที่ `save: true` |
| `src/chassis.py`, `src/PID.py` | `move_to` ใช้ position/attitude ที่ยังสด, แปลงความเร็วจากกรอบโลกเป็นกรอบรถ, จำกัดความเร็วและหยุดรถเมื่อจบหรือผิดพลาด; `PIDController` คำนวณค่าควบคุม |
| `src/dashboard.py`, `dashboard/index.html` | server ภาพสดและ telemetry; `/api/status` ส่งค่าล่าสุด, `/api/history` ส่งจุดใหม่พร้อมชื่อคอลัมน์, `/video` ส่ง MJPEG |
| `src/slam.py` | `CellWallGrid` เก็บ occupancy ของขอบช่องที่ใช้ร่วมกัน; `OccupancyGridSLAM` ฉายลำแสง ToF เดี่ยว โดยใช้ pose, มุม yaw ของ gimbal และ offset จากแกน yaw; `SlamWorker` เลือก ToF channel ตาม config, อ่าน sample timestamp ใกล้กันและทำงานเบื้องหลัง |
| `src/explorer.py` | `DFSExplorer` หัน gimbal ดูเพื่อนบ้านครบ 4 ทิศเมื่อเข้าช่องใหม่, รอ angle telemetry, action สำเร็จ และ ToF scan ใหม่ที่ตรงทิศ; ก่อนเดินไปช่องใหม่ตรวจ ToF เทียบเกณฑ์กำแพง 300 มม. อีกครั้ง ทางกลับใช้ขอบเปิดที่บันทึกไว้โดยไม่หมุนตรวจ; การเคลื่อนที่ใช้ `ChassisController.move_to()` พร้อมจำกัดความเร็ว และหยุดเมื่อ telemetry/status/ทางกลับไม่ผ่านเกณฑ์ |
| `src/run_review.py`, `review/index.html`, `review.py` | อ่าน run จาก CSV/summary, ส่งข้อมูลตัวอย่างให้กราฟ, ดาวน์โหลด CSV เต็ม, แสดงปัญหาบางประเภทและการเล่นย้อนหลัง |
| `tests/` | ทดสอบ config, PID, logger, controller, dashboard และ review ด้วย fake robot/ข้อมูลชั่วคราว |

## สัญญาข้อมูลที่ใช้ร่วมกัน

- `STREAMS[name]` ระบุชื่อ stream, อุปกรณ์ SDK, เมธอด subscribe/unsubscribe และลำดับคอลัมน์ ลำดับนี้ต้องตรงกับ tuple ที่ callback ส่งมาและแถว CSV
- `logging.streams.<name>` มี `enabled`, `save`, `frequency_hz`; `enabled` เปิด subscription และ history ส่วน `save` เพิ่ม CSV (ต้องเปิด `enabled` ด้วย) position/attitude ต้องเปิดเมื่อ mission ทำงาน
- `SensorLogger.get_latest(name, max_age_s)` คืน tuple หรือ `None` หากยังไม่มี/เก่าเกินกำหนด `get_history_since(after_id)` คืน `{"cursor": id, "streams": {name: [(id, elapsed_s, values), ...]}}` โดย history มีเพดานต่อ stream
- `Dashboard.snapshot()` ส่ง `streams` ของ stream ที่เปิด, `dropped_csv_rows`, `camera_ready`, `camera_error`, `mission_status`; ตัวเลข/อาร์เรย์จาก tuple ถูก JSON แปลงเป็น array และค่าเก่าเกิน 2 วินาทีเป็น `null`
- `Dashboard.history()` เพิ่ม `columns` ให้ข้อมูล history หน้าเว็บใช้ชื่อคอลัมน์สร้างกราฟทั่วไปอัตโนมัติ; API ใช้ `since` เป็น sample id รวมทุก stream
- เมื่อบันทึกข้อมูล: `data/raw/<timestamp>/<stream>.csv` มี `timestamp,elapsed_s,<columns...>`; `run_summary.json` เขียนเมื่อหยุดปกติ และเก็บสถานะ, error, จำนวนแถว, แถวตกหล่น, ค่า stream และข้อผิดพลาด logger
- `RunStore.load_run()` ส่ง `streams.<name> = {columns, samples, total_rows, displayed_rows, gap_count, largest_gap_s, first_s, last_s}` โดย `samples` เป็น `[row_number, elapsed_s, values]`; หน้า review สร้างกราฟทั่วไปจาก CSV ที่มีอยู่
- `GET /api/map` ส่ง `robomaster-occupancy-grid` version 1: `resolution_m`, `width`, `height`, `origin`, `data` แบบแถว y เพิ่มจากล่างขึ้นบน (`-1` ไม่รู้จัก, `0` ว่าง, `100` สิ่งกีดขวาง), `pose`, `trajectory`, `counts`, `exploration` และ metadata `sensor_model` (ช่อง ToF, offset/pivot, yaw offset และมุม gimbal ของ scan ล่าสุด)
- Canvas บน dashboard วาด +X ขึ้นเป็นทิศเหนือ, −X ลงเป็นทิศใต้ และ +Y ไปขวาเป็นทิศตะวันออก ทั้งเซลล์แผนที่ เส้นทาง หุ่น และแนว ToF ใช้การฉายพิกัดเดียวกัน การหมุนภาพนี้ไม่เปลี่ยนลำดับข้อมูลใน JSON/ROS หรือคำสั่งเคลื่อนที่
- `GET /api/map/export?format=json` ส่งเอกสาร JSON เดียวกันเป็นไฟล์; `format=png` ส่งภาพ occupancy ขาว/เทา/ดำโดย +X ขึ้นและ +Y ไปขวา; `format=ros` ส่ง ZIP ของ `map.yaml`/`map.pgm`/`slam.json`; `POST /api/map/import` รับ JSON format/version เดียวกันและแทน map ในหน่วยความจำเมื่อ SLAM worker หยุด
- แผนที่บันทึกอัตโนมัติตาม `exploration.map.save_path` พร้อม PNG ชื่อเดียวกันคนละนามสกุล เช่น `data/maps/latest.json` กับ `data/maps/latest.png` และบันทึก `map.png` ใน `data/raw/<run>/` เพื่อเก็บภาพของแต่ละรัน ทั้งเมื่อจบปกติและหยุดรัน; โหลด JSON ตอนเริ่มด้วย `exploration.map.load_path` ได้เมื่อกรอบพิกัด SDK สอดคล้องกับแผนที่เดิม

## ส่วนที่แสดงผลเฉพาะทาง

กราฟของ stream ตัวเลขใหม่มักขึ้นใน dashboard สดและ review ได้ผ่าน metadata/CSV เดิม แต่ข้อมูลหลักยังมีตำแหน่งเฉพาะใน UI:

| ข้อมูล | Dashboard สด | Review |
| --- | --- | --- |
| position/attitude | แผนที่, ทิศรถ, ความเร็วประมาณ, พิกัด | แผนที่, เวลาเลือก, ความเร็วประมาณ |
| gimbal | มุมบนภาพกล้องและทิศบนแผนที่ | มุมและทิศบนแผนที่ |
| ToF ช่องที่ตั้งใน `exploration.sensor.tof_channel` | ตัวเลขบนภาพกล้องและตำแหน่งหัวเซนเซอร์บนแผนที่ | CSV มีครบสี่ช่องจาก SDK; review แสดงช่อง 0 เป็นค่าเริ่มต้น |
| status | กราฟทั่วไป | เหตุ picked up/slip/impact/roll over |
| camera | MJPEG สดใน RAM | ไม่มีภาพย้อนหลัง |
| mission | ข้อความ `mission_status` ใน RAM | สถานะ/error ระดับ run ใน summary |
| exploration | กริดกำแพงสี่ด้าน, occupancy ละเอียด, trajectory, DFS status | กริดสุดท้ายใน summary และตารางกำแพงใน review; JSON export มีทั้งสองกริด |

ดังนั้นการมีกราฟใหม่ไม่ได้ยืนยันว่า dashboard สื่อความหมายของระบบใหม่ครบ โดยเฉพาะสถานะ mission, ความปลอดภัย, หน่วย และเหตุผิดปกติ

## ขอบเขตและข้อควรระวัง

- ระบบเคลื่อนที่ใช้ odometry แบบสัมบูรณ์จาก `sub_position(cs=0)` หน่วยเมตร และ yaw จาก `sub_attitude` หน่วยองศา `drive_speed` ใช้กรอบตัวรถ ไม่มีกลไกหลบสิ่งกีดขวางหรือวางแผนอัตโนมัติ
- ระหว่าง DFS ตั้ง `exploration.heading_source: gimbal` เพื่อใช้ yaw ของ chassis ที่อนุมานจาก `gimbal.yaw_ground_deg - gimbal.yaw_deg` ในการคุม PID และเล็งทิศสแกน ปรับศูนย์กับ `sub_attitude` ครั้งแรก ข้อมูล gimbal ต้องสด; ถ้าขาดจะหยุดการเดิน ค่า `attitude` ยังใช้ใน SLAM pose/แผนที่ และตั้ง `heading_source: attitude` เพื่อกลับไปใช้ตัวเดิมได้ ตัวควบคุม yaw ชดเชยต่อเนื่องระหว่างเคลื่อน แม้ความคลาดน้อยกว่าค่า arrival tolerance
- `sample_timeout_s` ใช้กับ pose ของ controller; snapshot ของ dashboard ใช้เกณฑ์ 2 วินาที หากเพิ่มข้อมูลที่มีผลต่อคำสั่งขับ ต้องกำหนดเกณฑ์ความสดและพฤติกรรมเมื่อข้อมูลหายเอง
- History ใน RAM มีขอบเขตและอาจสูญจุดเก่าหากผู้ดูดึงช้า CSV ใช้ queue จำกัด; เมื่อเต็มจะข้ามแถวแต่ค่าล่าสุดยังอัปเดต Review อ่านเฉพาะ stream ที่บันทึกไว้
- Dashboard สดเปิดกล้องเมื่อ `dashboard.enabled: true`; หน้า review ไม่มีภาพย้อนหลัง ค่า `host` เริ่มต้นเป็น localhost และเว็บยังไม่มีระบบล็อกอิน
- การทดสอบใน `tests/` ไม่ครอบคลุมหุ่นยนต์จริง การปรับ PID สภาพพื้น และการส่งภาพผ่าน Wi-Fi จริง
- หุ่นยนต์นี้ใช้ ToF เดียวบน gimbal; แต่ละ scan วัดหนึ่งแนวและสะสมเป็น occupancy grid ตาม odometry การ scan matching เดิมต้องมีอย่างน้อยสอง endpoint ใน scan เดียว จึงไม่แก้ drift ในโหมด ToF เดี่ยว และไม่ใช่การ relocalize ทั่วแผนที่
- `exploration.sensor.tof_channel` เป็นดัชนีข้อมูล SDK แบบเริ่มนับจาก 0 (ค่าเริ่มต้น 0); CSV ใหม่ใช้ `tof_0_mm`–`tof_3_mm` ส่วน CSV เก่าที่ใช้ `tof_1_mm`–`tof_4_mm` ยังเปิดใน review ได้ โดยค่าตัวแรกหมายถึงช่อง 0 เหมือนกัน `offset_from_yaw_axis_m` ตั้งเป็น 0.075 m ตามระยะจากแกน yaw ที่ผู้ใช้ให้ และ `offset_yaw_deg: 0` สมมติว่า offset อยู่แนวเดียวกับเลนส์ ส่วน `pivot_x_m/pivot_y_m` ยังตั้งต้นเป็นศูนย์และต้องปรับตามตำแหน่งแกนจริงจากจุดกลางรถก่อนใช้บนฮาร์ดแวร์
- SLAM ไม่กำหนดระยะ ToF สั้นสุด/ไกลสุด; รับค่าบวกที่เป็นตัวเลข finite และข้ามค่า 0 ชั่วคราวโดยรอ scan ใหม่ ค่าระยะใกล้ใน `robot_clearance_m` ยังเป็น scan ที่ใช้ได้ แต่ไม่เพิ่ม occupancy ใต้ตัวรถ และปลายลำแสงที่อยู่นอกกริดจะถูกตัดที่ขอบกริดโดยไม่เพิ่มผนังเทียม Dashboard ระบุเมื่อค่าล่าสุดอยู่ในเขตกันตัวรถ ขณะที่ DFS ใช้เกณฑ์กำแพง 300 มม. จาก ToF ใหม่ก่อนสั่งเดิน
- การอ่าน ToF, gimbal, pose และ status ต้องสดและมี timestamp ใกล้กัน; DFS จะไม่ขับเมื่อมุม gimbal ยังไม่ถึงเป้าหมาย, ไม่มี scan ใหม่ตามทิศ, telemetry ขาด หรือ safety flag ทำงาน
- DFS ใช้ `CellWallGrid` ใน `src/slam.py` เก็บขอบสี่ด้านของช่องและค่า ToF ล่าสุดจากแต่ละทิศ โดยผลวัดตรงไม่ถูกลำแสงไกลเขียนทับ เลือกข้ามเฉพาะขอบ `open` ที่สแกนจากช่องปัจจุบันด้วย ToF มากกว่า `wall_threshold_mm` แล้วสแกนยืนยันทิศนั้นใหม่ แผนที่ละเอียดคงไว้สำหรับดูและ export แต่ไม่ใช้จุด occupancy ระหว่างหมุนหัวมาสร้างกำแพงกริด รายละเอียด schema และการฉายรังสีอยู่ใน [WALL_GRID.md](WALL_GRID.md)
- หลังสแกนสี่ทิศครั้งแรกในช่อง DFS ใช้ผนังที่วัดตรงจัดระยะกึ่งกลาง chassis ถึงผนังเป้าหมาย 0.25 m (`exploration.alignment.wall_distance_m`) โดยชดเชย offset ของหัว ToF มีผนังสองด้านตรงข้ามจะเล็งกึ่งกลางระหว่างผนัง มีด้านเดียวจะเล็งระยะเป้าหมาย และไม่มีผนังจะไม่ขยับ หากผนังสองฝั่งชิดเกินระยะเป้าหมายรวมกันหรือการแก้ตำแหน่งเกิน `max_shift_m` จะหยุดพร้อม error หลังขยับจะสแกนผนังที่ใช้ซ้ำเพื่อตรวจผล จุดหมายแต่ละช่องที่จัดแล้วถูกใช้ตอนเดินกลับ โดยไม่สแกนซ้ำเมื่อกลับเข้าช่องเดิม dashboard และ review แสดงผล alignment
- ตั้ง `exploration.alignment.enabled: false` เพื่อปิดเฉพาะการขยับจัดกลางช่อง DFS ยังสแกนกำแพง ตัดสินทางเดิน และเดินกลับตามปกติ dashboard/review แสดงว่าปิด alignment
- SDK Python 3.8 ที่ติดตั้งกำหนด `GimbalMoveAction` ของ `moveto()` เป็น `COORDINATE_YCPN` (`yaw CAR`) ดังนั้น yaw คำสั่งเทียบ chassis เช่นเดียวกับช่อง `yaw_deg` ใน `sub_angle` ส่วน `yaw_ground_deg` เป็นอีกกรอบที่ห้ามนำมาบวกเข้ากับคำสั่ง DFS เลือกมุมสมมูลในช่วง SDK `[-250°,250°]` ที่ใกล้ relative yaw ปัจจุบันสุด และตรวจมุม scan ในกรอบเดียวกันก่อนอนุญาตให้เดิน
- DFS ใช้ `gimbal.moveto(yaw=0)` เมื่อสแกนตรงหน้าและ `moveto()` กับทิศอื่น เพื่อไม่ให้ `recenter()` สั่ง pitch เทียบ chassis ค้าง รอ yaw/pitch telemetry และ ToF scan ใหม่ที่ตรงทิศ **และ** SDK action สำเร็จก่อนตรวจทางเดินหรือออกคำสั่ง gimbal ถัดไป ไม่มีเวลาหมดอายุสำหรับการรอข้อมูลเริ่มต้น, action, scan หลังเดิน หรือก้าว DFS; dashboard แสดง `waiting_gimbal_action` เมื่อมุมและ ToF พร้อมแต่ action ยังทำงาน และ `waiting_slam_scan` หลังเดิน ข้อมูลเซนเซอร์ที่ไม่สดหรือ SDK action ที่ล้มเหลวยังคงหยุดการสำรวจ ส่วน `exploration.max_sample_age_s` คืออายุข้อมูลที่ยอมรับได้ ไม่ใช่เวลาสูงสุดของงาน

อ้างอิง API ตำแหน่ง/ทิศ/ระยะจาก [RoboMaster Python SDK guide](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/robomaster.html) และสเปกช่วงระยะ/FOV ของ ToF จาก [คู่มือ RoboMaster EP](https://dl.djicdn.com/downloads/ROBOMASTER_EP/20220429UM/RoboMaster_EP_User_Manual_v1.2_EN_1.pdf)
