# เพิ่มระบบโดยเชื่อมกับของเดิม

ใช้เอกสารนี้ทุกครั้งที่เพิ่มเซนเซอร์ พฤติกรรมควบคุม การตัดสินใจ หรือหน้าแสดงผล อ่าน [ภาพรวมสถาปัตยกรรม](ARCHITECTURE.md) ก่อนเริ่ม

## ตัดสินใจจุดเชื่อม

1. ระบุข้อมูลเข้า/ออก หน่วย ความถี่ อายุข้อมูลที่ยอมรับ และผลเมื่อข้อมูลหาย
2. ตรวจว่าข้อมูลมีอยู่ใน `STREAMS` และ `SensorLogger.get_latest()`/`get_history_since()` หรือไม่ ถ้ามี ให้ใช้ชุดเดิมก่อน หลีกเลี่ยงการ subscribe SDK ซ้ำ
3. ถ้าเป็นการเคลื่อนที่ ให้ใช้ `ChassisController`/`PIDController` และ config motion ที่มีอยู่เมื่อพฤติกรรมตรงกัน หากความต้องการใหม่ต่างจริง ให้เพิ่มชั้นควบคุมที่รับ logger และส่งคำสั่งผ่านทางเดียวที่ชัดเจน ห้ามให้สอง controller สั่ง `drive_speed()` พร้อมกัน
4. ถ้าเป็นข้อมูลที่ต้องตรวจย้อนหลัง ให้ใช้รูปแบบ CSV/run summary เดิม ประเมินว่า metadata คอลัมน์กับกราฟทั่วไปเพียงพอหรือควรเพิ่มการตีความ/เหตุการณ์ใน `RunStore`
5. ถ้าต้องเพิ่มโมดูลใหม่ ให้เขียนเหตุผลสั้น ๆ ในเอกสารฟีเจอร์หรือ PR: จุดใดของระบบเดิมใช้ไม่ได้, อินเทอร์เฟซใหม่เชื่อมกับ logger/controller/main อย่างไร, ใครเริ่มและหยุดโมดูล

## จุดแก้ตามชนิดงาน

| ชนิดงาน | จุดเริ่มต้น | จุดที่ต้องตรวจต่อ |
| --- | --- | --- |
| เซนเซอร์ SDK ใหม่ | เพิ่ม entry ใน `src/logger.py:STREAMS`; รองรับรูปข้อมูล callback ถ้าจำเป็น | `config/settings.yaml`, `src/config_loader.py`, tests, กราฟ/หน่วยใน `dashboard/index.html`, CSV และ `review/index.html` |
| ข้อมูลคำนวณจาก stream เดิม | อ่านจาก `SensorLogger` ครั้งเดียว; กำหนดหน่วย/อายุข้อมูล/แหล่งที่มา | เลือกเผยแพร่เป็น stream หรือ field สถานะที่มีสัญญาชัด; live dashboard, CSV/review หากต้องย้อนหลัง |
| พฤติกรรมขับรถหรือ mission | `src/chassis.py` หรือโมดูลที่ใช้ `ChassisController`; ประสานวงจรชีวิตใน `main.py` | config, การหยุดเมื่อ error/timeout, mission status ใน dashboard, สถานะผลลัพธ์ใน summary/review |
| กฎแจ้งเตือนหรือเหตุผิดปกติ | ใช้ค่าที่ logger เก็บและกำหนด threshold ใน config | สถานะและเวลาเกิดบน dashboard สด, การบันทึก/ตรวจย้อนหลังใน review ถ้าต้องสืบเหตุ |
| UI ใหม่ | ใช้ `/api/status` และ `/api/history` ก่อนเพิ่ม endpoint | ชื่อ/หน่วย/สถานะไม่มีข้อมูล, การเปิดปิด stream, หน้า review เมื่อข้อมูลถูกบันทึก |
| SLAM/การสำรวจ | ใช้ `SensorLogger` stream `position`, `attitude`, `gimbal`, `tof`, `status`; เลือก ToF channel ตาม config; `DFSExplorer` สั่ง gimbal scan และเรียก `ChassisController` เดิม | offset จากแกน yaw, ตำแหน่ง pivot, อายุข้อมูลและการซิงก์, safety status, status DFS, `/api/map`, dashboard overlay และ import/export JSON/ROS |

## งานด้าน dashboard ที่ต้องทำพร้อมฟีเจอร์

- ระบุสิ่งที่คนคุมหุ่นยนต์ต้องเห็นทันที: ค่าล่าสุด หน่วย สถานะพร้อมใช้/เก่า/หาย และสถานะของ mission ถ้ามีผลต่อการทำงาน
- ตรวจทั้ง `src/dashboard.py` และ `dashboard/index.html` กราฟใหม่จะสร้างเองจาก stream ที่เปิดและมีข้อมูลตัวเลข แต่ card, map, overlay, warning และคำอธิบายหน่วยต้องเพิ่มเองเมื่อเป็นข้อมูลสำคัญ
- หากต้องย้อนดู ให้ตรวจ `src/run_review.py` และ `review/index.html` ด้วย: CSV ใหม่ขึ้นเป็นกราฟอัตโนมัติ แต่ค่าเด่นบนหน้า review และ issue เฉพาะทางต้องกำหนดเอง
- หากเป็นฟีเจอร์ที่ไม่ควรมีข้อมูลย้อนหลัง เช่น ภาพกล้องสด ให้ระบุข้อจำกัดชัดเจนในเอกสาร ไม่ทำให้หน้า review แสดงข้อมูลที่ไม่ได้บันทึก
- ตรวจการแสดงเมื่อ stream ถูกปิด, ยังไม่มี sample, sample เก่า, ค่าเป็น `null`, และเมื่อ CSV ไม่มีข้อมูลบางช่วง
- งาน SLAM ต้องรักษาหน่วยและกรอบพิกัดใน metadata, ปฏิเสธ scan ที่ไม่ครบ/ไม่สด, แสดง unknown แยกจาก free/occupied และตรวจว่า export เปิดกลับด้วย `load_dict()` ได้
- การเปลี่ยน DFS ต้องคงกติกาว่าเดินเฉพาะทิศที่ gimbal/ToF ตรวจใหม่และ ToF เกิน `exploration.wall_threshold_mm` และห้ามวิ่งย้อน stack เมื่อขอบแผนที่ทางกลับไม่เป็น `open`
- Alignment หลังสแกนช่องใหม่ใช้ระยะผนังจากกึ่งกลาง chassis โดยรวม offset หัว ToF และใช้ `ChassisController.move_to` ในกรอบพิกัด odometry เดียวกับจุดหมาย DFS ต้องเก็บเป้าที่จัดแล้วสำหรับทางกลับ หยุดหากระยะผนังสองฝั่งขัดกับเป้าหมายหรือจำเป็นต้องขยับเกิน `exploration.alignment.max_shift_m`; ตรวจ dashboard/review ให้เห็นระยะก่อนและหลังขยับ
- `exploration.alignment.enabled` ควบคุมเฉพาะ alignment; เมื่อ false ต้องคงการสแกนกำแพงและการเดิน DFS และแสดงสถานะว่าปิดบน dashboard/review
- `exploration.heading_source` เลือก yaw สำหรับ chassis PID และ DFS scan: `gimbal` คำนวณจาก yaw อ้างอิงพื้นลบ yaw เทียบ chassis ใน sample เดียวกันและปรับศูนย์กับ attitude ครั้งแรก; `attitude` ใช้ช่อง chassis เดิม ต้องตรวจความสดของ gimbal ก่อนเดินเมื่อเลือก gimbal และแสดงแหล่ง yaw บน dashboard/review

### ToF เดี่ยวบน gimbal

ใช้ stream `tof` เดิมซึ่งเก็บ SDK ทั้งสี่ช่องไว้ใน CSV/history แล้วเลือก `exploration.sensor.tof_channel` ด้วยดัชนี 0–3 เฉพาะตอนทำ mapping ไม่สร้าง subscription ใหม่ CSV ใหม่ใช้ชื่อ `tof_0_mm`–`tof_3_mm`; review ยังอ่าน CSV เก่าที่ใช้ชื่อ 1–4 ได้ มุมยิงคำนวณจาก yaw รถ + relative yaw ของ gimbal + sensor yaw offset; จุดเริ่มลำแสงคือ pivot ที่ตั้งในกรอบรถ บวก `offset_from_yaw_axis_m` ตาม `offset_yaw_deg` DFS เล็งไปยังช่องที่ต้องตรวจ รอ telemetry มุมและ ToF scan ใหม่ที่ตรงทิศ แล้วจึงประเมินทางก่อนเรียก chassis controller เดิม

SDK Python 3.8 ที่ใช้ในโปรเจคสร้าง `GimbalMoveAction` ของ `moveto()` ด้วย `COORDINATE_YCPN` (`yaw CAR`) จึงรับ yaw เทียบ chassis เช่นเดียวกับ `gimbal` sample ช่องที่ 2 (`yaw_deg`) ไม่ใช้ช่องที่ 4 (`yaw_ground_deg`) คำนวณคำสั่ง เลือกมุมสมมูลใน `[-250°,250°]` ที่ใกล้ relative yaw ปัจจุบันที่สุด โดยเฉพาะทิศหลัง `+180°` กับ `−180°` และเพิ่มเวลาเคลื่อนตามมุมที่ต้องหมุนจริงก่อนรอ scan ใหม่

เมื่อ scan เป้าหมายเป็นกลาง ใช้ `moveto(yaw=0)` ของ SDK เพื่อเล็งตรงหน้าโดยไม่เรียก `recenter()` ที่ควบคุม pitch เทียบ chassis ด้วย คงใช้ action object ของ SDK และ `SensorLogger` เดิม ไม่สร้างโมดูลควบคุมหรือ subscription เพิ่ม ต้องรอ action สำเร็จพร้อมกับ yaw/pitch telemetry และ ToF scan ใหม่ที่ตรงทิศก่อนออกคำสั่ง gimbal ถัดไปหรือสั่ง chassis; ค่า yaw ตรงเป้าอย่างเดียวไม่หมายความว่า action ถูกปลดจาก dispatcher แล้ว เมื่อเข้าช่องใหม่ DFS ต้องสแกนครบ 4 ทิศพร้อมแสดง `scanning_1_of_4` ถึง `scanning_4_of_4` บน dashboard แล้วสแกนทิศที่จะไปช่องใหม่ซ้ำเพื่อยืนยันทางล่าสุด เมื่อเดินย้อน stack ไปช่องที่เคยเยี่ยม ให้ตรวจขอบร่วมในกริดว่า `open` โดยไม่หมุน gimbal ซ้ำ และเมื่อกลับมาถึงช่องเดิมไม่สแกนสี่ทิศซ้ำ

กรณีรัน `20260925_214708_411803`: yaw และ ToF ตรงทิศแล้ว แต่ `recenter()` ไม่รายงานว่าจบภายในเวลา; pitch ยังคลาดจากศูนย์และข้อมูลที่บันทึกไม่พอชี้ชัดว่าแกน pitch ติดหรือ completion push ของ SDK หาย การแก้ใช้ `moveto(yaw=0)` และรอ action/scan ตามเงื่อนไขโดยไม่ตั้งเวลาหมดอายุ รวมถึงการรอข้อมูลเริ่มต้น, scan หลังเดิน และการเดิน DFS ผ่าน `ChassisController` ด้วย ทดสอบ action ที่จบช้า, ล้มเหลว และ scan ที่มาก่อน action จบ ห้ามเดินหรือสั่ง gimbal action ใหม่จน action เดิมจบอย่างถูกต้อง; dashboard แสดง `waiting_gimbal_action` หรือ `waiting_slam_scan` ระหว่างรอ และ run review แสดง error จาก summary เมื่อมีความล้มเหลวจริง `exploration.max_sample_age_s` ยังตรวจอายุข้อมูลเพื่อกันการใช้ข้อมูลเก่า ห้ามเพิ่ม timeout งานในเส้นทาง SLAM/DFS/gimbal เองจนกว่าผู้ใช้จะระบุว่าต้องการ

ToF ไม่มีเกณฑ์ระยะสั้นสุด/ไกลสุดใน config; รับค่าบวก finite ทุกค่า ค่า 0/ค่าผิดรูปแบบถูกข้ามและรอค่าใหม่ ไม่ทำให้ SLAM ล้มทันที; เมื่อไม่มีค่าที่ใช้ได้นานเกิน `max_sample_age_s` ระบบหยุดเพราะข้อมูลไม่สด ระยะภายใน `robot_clearance_m` ไม่เขียนทับพื้นที่ใต้หุ่นเป็นกำแพง; ระยะที่ไกลกว่ากริดจะถูกตัดที่ขอบแผนที่และไม่ถือว่าขอบเป็นกำแพง DFS ใช้ ToF ใหม่ไม่เกิน `exploration.wall_threshold_mm` (ค่าเริ่มต้น 300 มม.) เป็นกำแพง และมากกว่าเกณฑ์เป็นทางเปิด โดยไม่คำนวณระยะเผื่อขนาดตัวหุ่นก่อนเรียก `ChassisController` หน้า dashboard และ review แสดงสถานะผนัง/ทางเปิด/ยังไม่รู้ พร้อมค่าที่วัดและเกณฑ์

หน้า dashboard และ `/api/map` ใช้ `sensor_model` metadata ชุดเดียวกันเพื่อแสดง channel, offset และทิศหัว ToF; หากเปลี่ยนช่อง, offset, pivot หรือ yaw alignment ให้ตรวจทั้ง overlay กับ JSON export/import. ค่าเริ่มต้นสมมติว่า offset 7.5 ซม. อยู่แนวเลนส์ (`offset_yaw_deg: 0`). ค่า `pivot_x_m/pivot_y_m` ต้องวัดจากหุ่นจริง เพราะระยะ 7.5 ซม. ที่ทราบอยู่แล้วเริ่มจากแกน gimbal ไม่ได้ระบุตำแหน่งแกนเทียบจุดกลาง chassis.

การหยุด `SlamWorker` บันทึก PNG คู่กับ JSON จาก `exploration.map.save_path` และ `map.png` ในโฟลเดอร์ log ของรัน; PNG เป็นภาพ occupancy ที่ +X อยู่ด้านบนและ +Y อยู่ด้านขวาเหมือน dashboard ใช้สีขาวแทนว่าง เทาแทนยังไม่รู้ และดำแทนกำแพง ต้องตรวจ PNG ที่บันทึกและ `/api/map/export?format=png` เมื่อต้องเปลี่ยนการวางแกนหรือสี

## ตัวอย่างการเพิ่มเซนเซอร์

หาก SDK มี stream ระยะทางใหม่ ให้ดูว่า `tof` เดิมให้ข้อมูลเดียวกันหรือไม่ ถ้าครอบคลุม ให้ใช้ `logger.get_latest("tof", max_age_s=...)` และต่อยอดการแสดงผลจาก stream เดิม หากเป็นคนละข้อมูลจริง ให้เพิ่มชื่อใน `STREAMS` พร้อมลำดับคอลัมน์/หน่วย, ตั้งค่า stream ใน YAML, ตรวจ config, เขียน test ด้วย fake SDK callback แล้วเปิด dashboard เพื่อตรวจค่าล่าสุดและกราฟ ถ้าต้องสืบเหตุย้อนหลัง ให้เปิด `save` และตรวจ CSV, review และเกณฑ์แจ้งเตือนที่เกี่ยวข้อง

## เกณฑ์ก่อนจบงาน

- ฟีเจอร์ใช้แหล่งข้อมูล/คำสั่งร่วมเดิมเท่าที่เหมาะสม และอธิบายเหตุผลเมื่อแยกส่วนใหม่
- วงจรเริ่ม/หยุดและการหยุดรถเมื่อผิดพลาดครบถ้วน ไม่มี subscription หรือ controller ซ้ำโดยไม่จำเป็น
- config และเอกสารระบุหน่วย พิกัด ความถี่ อายุข้อมูล ค่าเริ่มต้น และพฤติกรรมเมื่อข้อมูลหาย
- Dashboard สดแสดงสถานะที่ใช้ตัดสินใจได้จริง และ review สอดคล้องกับข้อมูลที่บันทึก
- เพิ่ม test ที่พิสูจน์สัญญาหรือกรณีผิดพลาดสำคัญ แล้วรัน `python -m unittest discover -s tests -v` ใน virtualenv Python 3.8; งานที่พึ่งฮาร์ดแวร์ให้ตรวจบน RoboMaster EP จริงด้วย

## กริดกำแพงสี่ด้าน

ใช้ `CellWallGrid` ใน `src/slam.py` ร่วมกับ snapshot ของ `DFSExplorer` ผลวัดขอบติดช่องโดยตรงมีสิทธิ์เหนือผลอนุมานจากลำแสงไกล ห้ามสร้าง subscription ใหม่หรือกริดที่แยกจาก export ของแผนที่ อ่าน [WALL_GRID.md](WALL_GRID.md) ก่อนเปลี่ยนการจัดกำแพงลงขอบช่อง ต้องทดสอบว่าขอบร่วมตรงกันและข้อมูล unknown ไม่อนุญาตให้เดิน
