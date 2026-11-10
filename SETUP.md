# NMITSat_FCS Setup Guide

Flash and run the CanSat flight software on a **Teensy 4.1**.

## 1. Tools

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install **[Teensyduino](https://www.pjrc.com/teensy/td_download.html)** for Teensy 4.1 support
3. Board menu: **Teensy 4.1**
4. USB Type: **Serial**

## 2. Libraries (Library Manager)

| Library | Purpose |
|---------|---------|
| TinyGPS++ | GPS NMEA parse |
| Adafruit BME680 | Pressure / temp / humidity |
| Adafruit Unified Sensor | BME680 dependency |
| MPU6050 (Electronic Cats / Rowberg) | IMU |

`Wire`, `SPI`, and `SD` ship with Teensyduino.

## 3. Open & upload

```
firmware/nmitsat_fcs/nmitsat_fcs.ino
```

1. Wire sensors / XBee / GPS per **[docs/wiring.md](docs/wiring.md)**
2. Insert a FAT-formatted microSD in the Teensy 4.1 slot
3. Select the Teensy port → **Upload**
4. Open Serial Monitor at **9600** baud

You should see SD init messages, then CSV telemetry once the GPS gets a fix.

## 4. Ground station smoke test

1. Pair XBee with the GCS radio (same PAN / baud **115200**)
2. Confirm CSV lines arrive on the ground serial terminal
3. Send `a,b` then `c,d` to toggle the camera trigger pin
4. After flight (or bench run), eject SD and open `data.txt`

## 5. Pre-flight checklist

- [ ] GPS has sky view / fix (non-zero lat/lon and satellite count)
- [ ] BME680 altitude roughly matches local QNH (set `SEALEVELPRESSURE_HPA`)
- [ ] SD logging (`Data logged to SD`)
- [ ] XBee downlink matches USB Serial
- [ ] Camera trigger audible/visual ack on `a,b` / `c,d`
- [ ] Battery placeholder replaced or noted if still hard-coded

## 6. Troubleshooting

| Symptom | Check |
|---------|--------|
| SD init failed | Card format (FAT32), reseat in Teensy slot |
| No GPS packets | Antenna, Serial1 wiring (RX/TX not swapped), baud 9600 |
| BME680 halt in `setup` | I2C address / power / pull-ups |
| MPU warning only | Wiring; telemetry still runs with zeros-ish motion |
| No XBee data | Serial2 pins, baud 115200, XBee config |
| SD stops after one line (old sketch) | Use current firmware — file is flushed, not permanently closed |

## 7. Related repos

- **MGAS** — mono-gimbal attitude stabilizer actuator / demo used in the broader NMITsat work
