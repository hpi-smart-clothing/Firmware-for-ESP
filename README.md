# Firmware Data Structure

This firmware collects data form multiple Attinys via UART. Each Attiny collects the orrientation data from a BNO055 IMU Sensor. It sends them as JSON via serial and provides a BLE server with a subscription to get some of the orientation data. It also provides a service to controll 2 vibration motors via BLE.

---

## Packet Overview

- **Total Length:** `NUM_ATTINYS × 26` bytes  
- **Per-IMU Block:** 26 bytes, in this order:  
  1. Linear Acceleration (6 bytes)  
  2. Gravity           (6 bytes)  
  3. Gyroscope         (6 bytes)  
  4. Quaternion        (8 bytes)  

_All values are signed 16-bit integers (`int16_t`), little-endian (LSB first)._

---

## Per-IMU Byte Map

For IMU #1, bytes `0…25`; for IMU #2, bytes `26…51`; and so on.

| Offset | Bytes  | Field            | Raw Type  | Conversion to Float                          |
|:------:|:-------|:-----------------|:----------|:---------------------------------------------|
| 0      | 0–1    | Linear Acc X     | int16_t   | `raw * 0.00981f` → m/s²                      |
| 2      | 2–3    | Linear Acc Y     | int16_t   | `raw * 0.00981f` → m/s²                      |
| 4      | 4–5    | Linear Acc Z     | int16_t   | `raw * 0.00981f` → m/s²                      |
| 6      | 6–7    | Gravity X        | int16_t   | `raw * 0.00981f` → m/s²                      |
| 8      | 8–9    | Gravity Y        | int16_t   | `raw * 0.00981f` → m/s²                      |
| 10     | 10–11  | Gravity Z        | int16_t   | `raw * 0.00981f` → m/s²                      |
| 12     | 12–13  | Gyro X           | int16_t   | `raw / 16.0f`   → °/s                        |
| 14     | 14–15  | Gyro Y           | int16_t   | `raw / 16.0f`   → °/s                        |
| 16     | 16–17  | Gyro Z           | int16_t   | `raw / 16.0f`   → °/s                        |
| 18     | 18–19  | Quaternion W     | int16_t   | `raw / 16384.0f` → unit quaternion component |
| 20     | 20–21  | Quaternion X     | int16_t   | `raw / 16384.0f` → unit quaternion component |
| 22     | 22–23  | Quaternion Y     | int16_t   | `raw / 16384.0f` → unit quaternion component |
| 24     | 24–25  | Quaternion Z     | int16_t   | `raw / 16384.0f` → unit quaternion component |
## Vibration Command Format

The BLE interface for haptic feedback accepts a stream of 4-byte intervals:

- **Bytes 0–1**: Duration in milliseconds (uint16, little-endian)  
- **Byte 2**: Left motor intensity (0–255)  
- **Byte 3**: Right motor intensity (0–255)  

You may concatenate multiple intervals back-to-back in a single packet or send them in separate writes. Internally the firmware enqueues up to **16 intervals**; any beyond that are discarded.

### Example

To alternate both motors at full intensity for 1 s each, send the following 8-byte packet:

`03 E8 FF 00 03 E8 00 FF`

- `03 E8` → 1000 ms  
- `FF 00` → left=255, right=0  
- `03 E8` → 1000 ms  
- `00 FF` → left=0,   right=255  
