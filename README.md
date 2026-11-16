# ระบบคัดแยกวัตถุอัตโนมัติด้วย Mechatronics และ YOLO

โปรเจกต์นี้เป็นระบบคัดแยกวัตถุบนสายพานลำเลียงแบบอัตโนมัติ โดยใช้ Raspberry Pi 5 เป็นศูนย์กลางควบคุมการทำงาน เชื่อมต่อกับเซนเซอร์ตรวจจับวัตถุ กล้อง Intel RealSense กล้อง/ระบบประมวลผลภาพ YOLO และหุ่นยนต์ Dobot MG400 เพื่อหยิบและวางวัตถุตามผลการตรวจจับ

โปรแกรมทั้งหมดอยู่ในโฟลเดอร์ [`source code`](source%20code/) และรูปภาพประกอบอยู่ในโฟลเดอร์ [`image`](image/)

## ภาพรวมระบบ

![ภาพรวมการเชื่อมต่ออุปกรณ์](<image/Mechatronic Project.png>)

องค์ประกอบหลักของระบบมีดังนี้

| อุปกรณ์/ซอฟต์แวร์ | หน้าที่ |
|---|---|
| Raspberry Pi 5 | ประสานงานระบบ กล้อง เว็บ UI และการสื่อสารกับอุปกรณ์ภายนอก |
| Cytron Motion 2350 Pro / RP2350 | ควบคุมมอเตอร์สายพานและอ่านค่าเซนเซอร์ Start/End |
| Optex BGS-2S15N | ตรวจจับวัตถุที่เข้าสู่สายพานและวัตถุถึงตำแหน่งหยุด |
| Intel RealSense D435i | ส่งภาพสีและภาพความลึกสำหรับวิเคราะห์ขนาด/ความสูงของวัตถุ |
| OpenCV และ NumPy | แยกวัตถุจากภาพความลึก วัดขนาด และสร้างภาพ Crop |
| YOLO/YOLO-seg | ตรวจจับและจำแนกวัตถุจากภาพ Crop โดยทำงานผ่าน Docker ได้ |
| Dobot MG400 และ Vacuum Pump | หยิบวัตถุจากตำแหน่งคงที่และวางไปยังตำแหน่งเป้าหมาย |

## ภาพรวม Workstation

ภาพรวมการจัดวางอุปกรณ์ทั้งหมดในพื้นที่ทดลอง ตั้งแต่สายพานลำเลียง จุดตรวจจับ กล้อง ไปจนถึงตำแหน่งของ Dobot MG400

![พื้นที่การทดลอง](<image/Work Station.png>)

## ลำดับการทำงาน

![ลำดับการทำงานระบบอัตโนมัติ](<image/ขั้นตอนการทำงานระบบมีคาทรอนิกส์อัตโนมัติ.png>)

1. เซนเซอร์ตรวจพบวัตถุและเริ่มระบบ
2. กล้อง RealSense จับภาพวัตถุระหว่างเคลื่อนที่บนสายพาน
3. ระบบ OpenCV วิเคราะห์ภาพความลึกและเลือกเฟรมที่เหมาะสมที่สุด จากนั้นสร้างภาพ Crop
4. YOLO-seg วิเคราะห์ภาพ Crop เพื่อระบุประเภทและขอบเขตของวัตถุ
5. ระบบหยุดสายพานเมื่อวัตถุถึงตำแหน่ง End และ Dobot MG400 หยิบวัตถุไปวาง
6. ผู้ใช้ติดตามภาพ Color/Depth และสถานะการทำงานผ่าน Web Stream

## ขั้นตอนการตรวจจับวัตถุ

![ขั้นตอนการทำงานระบบตรวจจับวัตถุ](<image/ขั้นตอนการทำงานระบบตรวจจับวัตถุ.png>)

ระบบตรวจจับใช้ภาพความลึกเป็นหลัก โดย `detector.py` จะค้นหาบริเวณที่มีความแตกต่างจากระนาบสายพาน วัดขนาด/ความสูง และส่งผลให้ส่วนควบคุม Dobot ตรวจสอบก่อนสั่งหยิบ ส่วน YOLO ใช้ช่วยระบุวัตถุและปรับปรุงขอบเขตการตรวจจับเท่านั้น ไม่ได้เป็นตัวอนุมัติการหยิบโดยตรง

## โครงสร้างไฟล์

```text
.
├── README.md
├── image/
│   ├── Mechatronic Project.png
│   ├── ขั้นตอนการทำงานระบบตรวจจับวัตถุ.png
│   └── ขั้นตอนการทำงานระบบมีคาทรอนิกส์อัตโนมัติ.png
├── source code/
│   ├── Raspberry Pi/            # โปรแกรมควบคุมและประมวลผลบน Raspberry Pi
│   │   ├── piSerial.py          # เริ่มและควบคุมโปรเซสหลัก
│   │   ├── camera.py            # รับภาพ RealSense และทำงานอัตโนมัติ
│   │   ├── detector.py          # วิเคราะห์ภาพความลึกและวัดวัตถุ
│   │   ├── dobot.py             # ควบคุม Dobot MG400
│   │   ├── web_stream.py        # Web UI และภาพสตรีม
│   │   ├── yolo_detect.py       # ประมวลผล YOLO/YOLO-seg
│   │   ├── yolo_server.py       # HTTP server สำหรับ YOLO ใน Docker
│   │   ├── state_io.py          # อ่าน/เขียนไฟล์สถานะร่วมกัน
│   │   ├── *_settings.json      # ค่าตั้งต้นของอุปกรณ์
│   │   └── STATE_CONTRACT.md    # รูปแบบและ handshake ของ state files
│   ├── Pi Pico/                 # โปรแกรมควบคุมสายพานผ่าน Serial
│   │   └── conveyor.py          # โปรแกรมฝั่ง Raspberry Pi สำหรับควบคุมสายพาน
│   └── README.md                # คู่มือการติดตั้งและใช้งานเชิงเทคนิค
└── experiment/                  # ไฟล์ทดลองของโปรเจกต์
```

## ผลการทดลอง

ตัวอย่างภาพก่อน/หลังการตรวจจับ ภาพหน้าจอ Web Stream และวิดีโอการทำงานจริงอยู่ใน [หน้าผลการทดลอง](experiment/README.md)

## การสื่อสารระหว่างโปรแกรม

โปรแกรมฝั่ง Raspberry Pi แบ่งเป็นโปรเซสย่อยและสื่อสารกันผ่านไฟล์ JSON ในโฟลเดอร์ `state/`

```text
conveyor.py  -> conveyor_status.json -> camera.py / dobot.py
camera.py    -> camera_status.json และ detect_result.json
dobot.py     -> dobot_status.json และ dobot_last_cycle.json
camera.py    -> HTTP /predict -> yolo_server.py -> YOLO
```

การจับคู่ผลตรวจจับกับวัตถุใช้ `object_id` เดียวกันตลอดหนึ่งรอบการทำงาน เพื่อป้องกันการหยิบผลของวัตถุคนละชิ้น รายละเอียด schema และเงื่อนไขของไฟล์สถานะอยู่ใน [`STATE_CONTRACT.md`](source%20code/Raspberry%20Pi/STATE_CONTRACT.md)

## การติดตั้งเบื้องต้น

ระบบเป้าหมายคือ Raspberry Pi OS ที่เชื่อมต่อกับอุปกรณ์จริง

```bash
sudo apt install python3-serial python3-flask python3-opencv python3-numpy
pip install pyrealsense2
```

หากต้องการเก็บไฟล์สถานะไว้ใน RAM ให้กำหนดโฟลเดอร์ก่อนเริ่มโปรแกรม

```bash
export CELL_STATE_DIR=/dev/shm/cell_state
```

การติดตั้ง YOLO แบบแยกไว้ใน Docker และการตั้งค่าโมเดลอธิบายไว้ใน [`source code/README.md`](source%20code/README.md)

## การเริ่มระบบ

เข้าไปที่โฟลเดอร์โปรแกรมก่อน

```bash
cd "source code/Raspberry Pi"
```

เริ่มโปรแกรมอัตโนมัติทั้งระบบ โดย Dobot จะอยู่ในโหมดจำลองการทำงานเป็นค่าเริ่มต้น

```bash
python3 piSerial.py
```

ตัวเลือกที่ใช้บ่อย

```bash
python3 piSerial.py --only conveyor,camera  # ทดสอบสายพานและกล้องโดยไม่เริ่ม Dobot
python3 piSerial.py --only dobot            # เริ่มเฉพาะลูปควบคุม Dobot
python3 piSerial.py --dobot-live            # เปิดใช้งาน Dobot จริง
```

เปิดหน้าเว็บที่ `http://<ip-ของ-Raspberry-Pi>:5000` เพื่อดูภาพ Color/Depth สถานะกล้อง ผลตรวจจับ และสถานะสายพาน/หุ่นยนต์

## การตั้งค่าที่สำคัญ

- `camera_settings.json` — ความลึกสายพาน เกณฑ์ตรวจจับ พอร์ตเว็บ และการตั้งค่า YOLO
- `conveyor_settings.json` — ความเร็ว ทิศทาง เซนเซอร์ และเวลาหน่วงของสายพาน
- `dobot_settings.json` — IP ของ Dobot และโหมด `dobot_live`

ค่า `dobot_live` เริ่มต้นเป็น `false` เพื่อความปลอดภัย การสั่งให้หุ่นยนต์เคลื่อนที่จริงต้องเปิดทั้ง `--dobot-live` และตั้งค่า `"dobot_live": true` หลังตรวจสอบพื้นที่ทำงานและจุดหยิบเรียบร้อยแล้ว

## ขอบเขตของโปรเจกต์

- โค้ดใน repository นี้เป็นซอฟต์แวร์ฝั่ง Raspberry Pi และ Docker YOLO
- Firmware ของบอร์ด RP2350/Cytron เป็น dependency แยกต่างหาก ไม่ได้รวมอยู่ใน repository นี้
- `source code/Pi Pico/conveyor.py` สื่อสารกับ firmware ผ่าน Serial แต่ไม่ใช่ firmware ควบคุมมอเตอร์
- ควรทดสอบวัตถุทีละชิ้นในโหมดจำลองก่อนเชื่อมต่อ Dobot จริง

## เอกสารเพิ่มเติม

- [คู่มือการติดตั้งและใช้งานเชิงเทคนิค](source%20code/README.md)
- [รายละเอียด state files และ handshake](source%20code/Raspberry%20Pi/STATE_CONTRACT.md)
