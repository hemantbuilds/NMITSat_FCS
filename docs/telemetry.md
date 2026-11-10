# Telemetry & GCS protocol

## Downlink packet (CSV)

Each successful GPS+sensor sample is sent on **XBee Serial2**, printed on USB Serial, and appended to `data.txt` on the SD card.

Field order (as built in `telemetry()`):

| # | Field | Source |
|---|--------|--------|
| 1 | accel X (g) | MPU6050 |
| 2 | accel Y (g) | MPU6050 |
| 3 | accel Z (g) | MPU6050 |
| 4 | altitude (m) | BME680 barometric |
| 5 | temperature (°C) | BME680 |
| 6 | pressure (hPa) | BME680 |
| 7 | gyro X (°/s) | MPU6050 |
| 8 | gyro Y (°/s) | MPU6050 |
| 9 | gyro Z (°/s) | MPU6050 |
| 10 | latitude | GPS |
| 11 | longitude | GPS |
| 12 | date `D/M/Y` | GPS |
| 13 | battery % | placeholder constant (wire ADC later) |
| 14 | speed (m/s) | GPS |
| 15 | satellites | GPS |
| 16 | `humidity=<value>` | BME680 |

Example:

```text
0.02,-0.01,1.00,512.3,24.50,954.12,0.1,-0.2,0.0,12.971600,77.594600,10/11/2026,89,0.5,8,humidity=41.2
```

Packets are emitted about once per second when GPS sentences decode successfully.

## Ground commands (XBee → CanSat)

Commands are line-oriented strings on Serial2 (case-insensitive):

| Command | Action |
|---------|--------|
| `a,b` | Drive **ESP32_CAM_PIN** HIGH (camera start / enable) |
| `c,d` | Drive **ESP32_CAM_PIN** LOW (camera stop / disable) |

Any other string logs `Invalid command` on USB Serial. Buzzer patterns acknowledge TX telemetry (`telem_note`) and command receipt (`receive_note`).

## Mission logic (documented / partial)

From the original ops notes:

1. Power switch activates the electronic system
2. GCS can send calibrate / transmit / camera commands
3. Container logs sensor data to SD and relays via XBee
4. Camera footage saved on its own SD (ESP32-CAM side)
5. **Second parachute:** trigger near **600 m** apogee context, deploy near **500 m** — constants exist in firmware (`APOGEE_TRIGGER_M`, `CHUTE_DEPLOY_M`) awaiting actuator wiring
6. Post-landing buzzer until power-off — extend with a landing-detect state machine when ready
