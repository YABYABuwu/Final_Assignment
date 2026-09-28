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
| พฤติกรรมขับรถหรือ mission | `src/chassis.py` หรือโมดูลที่ใช้ `ChassisController`; ประสานวงจรชีวิตใน `main.py` | config รวม `motion.max_lateral_accel_m_s2`, การหยุดเมื่อ error/timeout, mission status ใน dashboard, สถานะผลลัพธ์ใน summary/review |
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
- Alignment หลังสแกนช่องใหม่ใช้ระยะผนังจากกึ่งกลาง chassis โดยรวม offset หัว ToF; ถ้ามีผนังสองฝั่งในแกนเดียวกัน ให้คำนวณ correction เป็น `(ระยะฝั่งบวก - ระยะฝั่งลบ) / 2` แล้วขยับให้ระยะเท่ากัน หากมีฝั่งเดียวจึงใช้ `wall_distance_m` คำสั่ง `ChassisController.move_to` ใช้ PID ในกรอบ odometry เดียวกับจุดหมาย DFS; `max_shift_m` จำกัดระยะต่อคำสั่ง PID ไม่ใช่ระยะรวมที่ต้องแก้ สแกนผนังที่ใช้ทุกฝั่งใหม่หลังแต่ละช่วง หากเลยเป้าหมายให้คำนวณทิศใหม่แล้วขยับกลับ เมื่อระยะคลาดไม่ลดลงให้บันทึก alignment เป็น `stalled` และไม่ขยับจัดกลางต่อในช่องนั้น; ข้อมูลเก่าต้องหยุดล้อแล้วรอข้อมูลสดใหม่ ส่วนทิศผิดต้องหยุดการเคลื่อนที่อย่างปลอดภัย ไม่มี emergency stop 20 ซม. ใน alignment
- Emergency stop 20 ซม. ใช้กับการเดินตามกริดทั้งไปและกลับ (`exploration.emergency_stop_distance_m`) ไม่ขึ้นกับ `alignment.enabled`; ก่อนเดินหัน ToF ไปทิศเดินหนึ่งทิศและรอ action/scan ใหม่ ระหว่างขยับ `stop_if` ตรวจ ToF สดกับ gimbal ที่หันตรงทิศ หากใกล้เกณฑ์ให้หยุดล้อและบันทึก pose จริงแยกจากเป้ากลางช่อง เกณฑ์ครึ่งทางใช้เลือกช่องที่น่าจะอยู่เท่านั้น. ยืนยันกลางช่องจาก pose ที่ใกล้เป้าภายใน tolerance เท่านั้น; ระยะผนังด้านเดียวไม่พอระบุกลางช่อง; ถ้ายืนยันไม่ได้ให้เก็บ `center_pending` และหยุดก่อนเดินต่อโดยไม่แก้ `cell_targets`. การกลับช่องเดิมไม่มีการสแกนสี่ด้านซ้ำ `exploration.last_motion_stop` แสดงเหตุ ระยะ ตำแหน่งจริง เป้ากลางช่อง และความคลาดบน dashboard/review
- `exploration.alignment.enabled` ควบคุมเฉพาะ alignment; เมื่อ false ต้องคงการสแกนกำแพงและการเดิน DFS และแสดงสถานะว่าปิดบน dashboard/review
- `exploration.heading_source` เลือก yaw สำหรับ chassis PID และ DFS scan: `gimbal` คำนวณจาก yaw อ้างอิงพื้นลบ yaw เทียบ chassis ใน sample เดียวกันและปรับศูนย์กับ attitude ครั้งแรก; `attitude` ใช้ช่อง chassis เดิม ก่อนเดินตามกริดแต่ละช่วง (รวมทางกลับ) และก่อนขยับจัดกลางช่อง DFS จับ yaw สดจากแหล่งที่เลือกแล้วส่งเป็นเป้าหมายคงที่ให้ PID ตลอดช่วงนั้น ต้องตรวจความสดของ gimbal ก่อนเดินเมื่อเลือก gimbal และแสดง yaw ที่จับได้บน dashboard/review

### ToF เดี่ยวบน gimbal

ใช้ stream `tof` เดิมซึ่งเก็บ SDK ทั้งสี่ช่องไว้ใน CSV/history แล้วเลือก `exploration.sensor.tof_channel` ด้วยดัชนี 0–3 เฉพาะตอนทำ mapping ไม่สร้าง subscription ใหม่ CSV ใหม่ใช้ชื่อ `tof_0_mm`–`tof_3_mm`; review ยังอ่าน CSV เก่าที่ใช้ชื่อ 1–4 ได้ มุมยิงคำนวณจาก yaw รถ + relative yaw ของ gimbal + sensor yaw offset; จุดเริ่มลำแสงคือ pivot ที่ตั้งในกรอบรถ บวก `offset_from_yaw_axis_m` ตาม `offset_yaw_deg` DFS เล็งไปยังช่องที่ต้องตรวจ รอ telemetry มุมและ ToF scan ใหม่ที่ตรงทิศ แล้วจึงประเมินทางก่อนเรียก chassis controller เดิม

SDK Python 3.8 ที่ใช้ในโปรเจคให้ `gimbal.moveto()` เป็น `COORDINATE_YCPN` ซึ่งรับ yaw เทียบ chassis แต่ pitch เทียบพื้น เมื่อผู้ใช้ต้องการหยุดการชดเชย pitch ของ chassis DFS จึงห่อ `GimbalMoveAction` ด้วย `COORDINATE_CAR` ใน `src/gimbal_control.py` แล้วส่งผ่าน action dispatcher เดิมของ SDK; public `moveto()` ไม่เปิดพารามิเตอร์ coordinate mode คำสั่ง yaw และ pitch ปัจจุบันเทียบ chassis ทั้งคู่ DFS เลือก yaw สมมูลใน `[-250°,250°]` ที่ใกล้ relative yaw ปัจจุบันที่สุด โดยเฉพาะทิศหลัง `+180°` กับ `−180°`

DFS ตรวจคำสั่งกับ telemetry ในกรอบเดียวกัน: yaw ช่องที่ 2 (`yaw_deg`) และ pitch ช่องที่ 1 (`pitch_deg`) ซึ่งเทียบ chassis ทั้งคู่ ไม่ใช้ `yaw_ground_deg` หรือ `pitch_ground_deg` ตัดสินว่าหัวตรงเป้า หน้า dashboard/review ยังแสดง pitch ทั้งสองกรอบเพื่อช่วยตรวจว่าหัวคงมุมเทียบรถหรือไม่; ค่าจาก telemetry ไม่ยืนยันแนวเลนส์ ToF ทางกล ต้องตรวจการติดตั้งจริงด้วย
ค่า yaw และ pitch ใช้ tolerance แยกกัน (`angle_tolerance_deg` และ `pitch_tolerance_deg`); ระหว่างรอ scan ให้ดู `exploration.scan_alignment` และข้อความมุมเป้าหมาย/จริงบน dashboard เพื่อทราบว่าค้างที่ action, มุม หรือ ToF. ค่า pitch telemetry ของรถคันนี้เปลี่ยนตาม yaw ได้ถึงประมาณ `±7.1°` จึงสอบเทียบ tolerance ไว้ `8°` และไม่ใช้การชดเชยแบบ relative. เมื่อ action สำเร็จแต่ telemetry สดนิ่งเกิน tolerance 3 sample จะลอง `moveto` เดิมอีกหนึ่งครั้ง แล้วหยุดพร้อม `MissionStop` หากยังผิดมุม (ไม่ใช้ scan ผิดทิศหรือเพิ่ม timeout)

เมื่อ scan เป้าหมายเป็นกลาง ใช้ action `COORDINATE_CAR` ที่ yaw=0 เพื่อเล็งตรงหน้าโดยไม่เรียก `recenter()` คงใช้ action object และ `SensorLogger` เดิม ไม่เปิด subscription เพิ่ม ต้องรอ action สำเร็จพร้อมกับ yaw/pitch telemetry และ ToF scan ใหม่ที่ตรงทิศก่อนออกคำสั่ง gimbal ถัดไปหรือสั่ง chassis; ค่า yaw ตรงเป้าอย่างเดียวไม่หมายความว่า action ถูกปลดจาก dispatcher แล้ว เมื่อเข้าช่องใหม่ DFS ต้องสแกนครบ 4 ทิศพร้อมแสดง `scanning_1_of_4` ถึง `scanning_4_of_4` บน dashboard แล้วสแกนทิศที่จะไปช่องใหม่หนึ่งครั้งก่อนเดินเพื่อยืนยันทางล่าสุด; ผล scan นี้ใช้เริ่มเดินทันที จึงไม่สั่ง scan ซ้ำอีกครั้งระหว่างเลือกทางกับเริ่มเดิน เมื่อเดินย้อน stack ไปช่องที่เคยเยี่ยม ให้ตรวจขอบร่วมในกริดว่า `open` แล้วหัน ToF ไปทางเดินกลับหนึ่งทิศเพื่อเฝ้าระยะฉุกเฉิน เมื่อกลับมาถึงช่องเดิมไม่สแกนสี่ทิศซ้ำ

ก่อนเริ่ม DFS ใช้ `recenter()` หนึ่งครั้งเมื่อ `auto_recenter` เปิด และรอ action completion โดยไม่ตั้ง timeout จากนั้นต้องได้ telemetry ใหม่ที่ yaw/pitch อยู่ใน tolerance จึงเริ่ม scan; หาก action ล้มเหลวหรือค่าหลังตั้งศูนย์ยังผิด ให้หยุดพร้อมเหตุผล. ระหว่าง scan ยังคงใช้ `moveto(yaw=0)` สำหรับด้านหน้าและรอ action/scan ตามเงื่อนไข รวมถึงการรอข้อมูลเริ่มต้น, scan หลังเดิน และการเดิน DFS ผ่าน `ChassisController`. ห้ามเดินหรือสั่ง gimbal action ใหม่จน action เดิมจบอย่างถูกต้อง; dashboard แสดง `centering_gimbal`, `waiting_gimbal_action` หรือ `waiting_slam_scan` ตามขั้นตอน

ToF ไม่มีเกณฑ์ระยะสั้นสุด/ไกลสุดใน config; รับค่าบวก finite ยกเว้น `65535` ซึ่งเป็นค่าที่ไม่ใช่ระยะจริง ค่า 0/65535/ค่าผิดรูปแบบถูกข้ามและรอค่าใหม่ แม้ callback ส่งค่าที่ใช้ไม่ได้ต่อเนื่องก็ไม่ทำให้ SLAM ล้ม; DFS แสดง `waiting_tof` และไม่ใช้ค่าดังกล่าวตัดสินทางเดิน ข้อมูล callback ที่ขาดหรือ timestamp ไม่สด/ไม่สัมพันธ์กับ pose และ gimbal ให้หยุดล้อและรอข้อมูลสดที่สัมพันธ์กันก่อนเดินต่อ ระยะภายใน `robot_clearance_m` ไม่เขียนทับพื้นที่ใต้หุ่นเป็นกำแพง; ระยะที่ไกลกว่ากริดจะถูกตัดที่ขอบแผนที่และไม่ถือว่าขอบเป็นกำแพง DFS รอ scan ใหม่คนละ timestamp หลัง gimbal action สำเร็จและหันตรงทิศครบ `exploration.tof_median_window` (ค่าเริ่มต้น 3) แล้วใช้ค่ามัธยฐานเทียบ `wall_threshold_mm`; ไม่คำนวณระยะเผื่อขนาดตัวหุ่นก่อนเรียก `ChassisController` หน้า dashboard และ review แสดงสถานะผนัง/ทางเปิด/ยังไม่รู้ พร้อมค่าที่วัด เกณฑ์ และจำนวน scan ที่ใช้

หน้า dashboard และ `/api/map` ใช้ `sensor_model` metadata ชุดเดียวกันเพื่อแสดง channel, offset และทิศหัว ToF; หากเปลี่ยนช่อง, offset, pivot หรือ yaw alignment ให้ตรวจทั้ง overlay กับ JSON export/import. ค่าเริ่มต้นสมมติว่า offset 7.5 ซม. อยู่แนวเลนส์ (`offset_yaw_deg: 0`). ค่า `pivot_x_m/pivot_y_m` ต้องวัดจากหุ่นจริง เพราะระยะ 7.5 ซม. ที่ทราบอยู่แล้วเริ่มจากแกน gimbal ไม่ได้ระบุตำแหน่งแกนเทียบจุดกลาง chassis.

การหยุด `SlamWorker` บันทึก PNG คู่กับ JSON จาก `exploration.map.save_path` และ `map.png` ในโฟลเดอร์ log ของรัน; PNG เป็นภาพ occupancy ที่ +X อยู่ด้านบนและ +Y อยู่ด้านขวาเหมือน dashboard ใช้สีขาวแทนว่าง เทาแทนยังไม่รู้ และดำแทนกำแพง ต้องตรวจ PNG ที่บันทึกและ `/api/map/export?format=png` เมื่อต้องเปลี่ยนการวางแกนหรือสี

PNG ของกริด DFS แยกเป็น `latest-grid.png` และ `map-grid.png` ในโฟลเดอร์รัน พร้อม endpoint `format=grid-png`; แสดง +X ขึ้น/+Y ขวาในแกนกริด สีส้มคือกำแพง เขียวคือขอบเปิด และเทาคือขอบยังไม่รู้ เส้นทาง stack สีฟ้า ไม่มีไฟล์กริดเมื่อยังไม่มี `cell_grid` ที่สแกนแล้ว

## ตัวอย่างการเพิ่มเซนเซอร์

IR กันชนหน้า/หลังใช้ `adapter` stream เดียวกันผ่าน `SensorLogger` โดยไม่เปิด subscription ซ้ำ: หน้า ID 4/port 2 = `io_8` (ซ้าย), ID 3/port 2 = `io_6` (ขวา); หลัง ID 4/port 1 = `io_7` (ซ้าย), ID 1/port 1 = `io_1` (ขวา, active IO 0). ยังปิด `front_ir.enabled` จนกว่าจะสอบเทียบ active IO ทั้งสองด้าน. Config loader ปฏิเสธการเปิด IR สองตัวบนพอร์ตเดียวกันหรือเปิดโดยไม่กำหนด polarity. Digital IR ทแยงข้างเดียวบอกไม่ได้ว่าวัตถุอยู่ด้านข้างหรือแนวเฉียง จึงใช้ staged recovery: attempt 1 หลบด้านตรงข้าม, attempt 2 เลื่อนออกจากปลายที่พบ (หลัง→หน้า, หน้า→หลัง), attempt 3 เลื่อนทแยงออกจากด้านที่พบ และ `z=0`. ความเร็วทแยง normalize ให้ขนาดรวมเท่ากับ `recovery_speed_m_s`. ก่อนหลบตรวจ IR ฝั่งตรงข้าม; เมื่อพบวัตถุให้หยุดและบันทึกสาเหตุ, เมื่อข้อมูลขาดให้หยุดล้อรอ. แต่ละ attempt ไม่เกิน `recovery_max_m`; ต้องมี IO ว่างทั้งสองด้านจาก callback ใหม่ครบ `recovery_clear_samples` ก่อนกลับไปยังเป้าหมายเดิม และหยุดเมื่อครบ `recovery_max_attempts` แล้วยังติด. Dashboard ส่งสถานะทั้งสองปลายและ run review อ่านค่าจาก config ที่บันทึกใน summary เพื่อแปล `adapter.csv`.

หาก SDK มี stream ระยะทางใหม่ ให้ดูว่า `tof` เดิมให้ข้อมูลเดียวกันหรือไม่ ถ้าครอบคลุม ให้ใช้ `logger.get_latest("tof", max_age_s=...)` และต่อยอดการแสดงผลจาก stream เดิม หากเป็นคนละข้อมูลจริง ให้เพิ่มชื่อใน `STREAMS` พร้อมลำดับคอลัมน์/หน่วย, ตั้งค่า stream ใน YAML, ตรวจ config, เขียน test ด้วย fake SDK callback แล้วเปิด dashboard เพื่อตรวจค่าล่าสุดและกราฟ ถ้าต้องสืบเหตุย้อนหลัง ให้เปิด `save` และตรวจ CSV, review และเกณฑ์แจ้งเตือนที่เกี่ยวข้อง

## เกณฑ์ก่อนจบงาน

- ฟีเจอร์ใช้แหล่งข้อมูล/คำสั่งร่วมเดิมเท่าที่เหมาะสม และอธิบายเหตุผลเมื่อแยกส่วนใหม่
- วงจรเริ่ม/หยุดและการหยุดรถเมื่อผิดพลาดครบถ้วน ไม่มี subscription หรือ controller ซ้ำโดยไม่จำเป็น
- config และเอกสารระบุหน่วย พิกัด ความถี่ อายุข้อมูล ค่าเริ่มต้น และพฤติกรรมเมื่อข้อมูลหาย
- Dashboard สดแสดงสถานะที่ใช้ตัดสินใจได้จริง และ review สอดคล้องกับข้อมูลที่บันทึก
- เพิ่ม test ที่พิสูจน์สัญญาหรือกรณีผิดพลาดสำคัญ แล้วรัน `python -m unittest discover -s tests -v` ใน virtualenv Python 3.8; งานที่พึ่งฮาร์ดแวร์ให้ตรวจบน RoboMaster EP จริงด้วย

## กริดกำแพงสี่ด้าน

ใช้ `CellWallGrid` ใน `src/slam.py` ร่วมกับ snapshot ของ `DFSExplorer` ผลวัดขอบติดช่องโดยตรงมีสิทธิ์เหนือผลอนุมานจากลำแสงไกล ห้ามสร้าง subscription ใหม่หรือกริดที่แยกจาก export ของแผนที่ อ่าน [WALL_GRID.md](WALL_GRID.md) ก่อนเปลี่ยนการจัดกำแพงลงขอบช่อง ต้องทดสอบว่าขอบร่วมตรงกันและข้อมูล unknown ไม่อนุญาตให้เดิน
