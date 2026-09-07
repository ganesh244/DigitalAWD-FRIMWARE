# LoRa Field Nodes V6 (T22D Lora1 to Lora5)

Battery-powered ultrasonic level sensors. Each node wakes every 20 to
25 minutes, measures the water level in its pipe, sends one LoRa frame
to the gateway and goes back to deep sleep. There is no receive path and
no acknowledgement; the gateway's per-node RSSI and silence counters are
the health signal.

## Which file goes on which node

| Folder | Device ID | First-boot stagger | Setup AP name |
|---|---|---|---|
| `AWD_LoRa_Node1_V6` | `T22D(Lora1)` | 0 min | LoRaTX-Lora1-Setup |
| `AWD_LoRa_Node2_V6` | `T22D(Lora2)` | 4 min | LoRaTX-Lora2-Setup |
| `AWD_LoRa_Node3_V6` | `T22D(Lora3)` | 8 min | LoRaTX-Lora3-Setup |
| `AWD_LoRa_Node4_V6` | `T22D(Lora4)` | 12 min | LoRaTX-Lora4-Setup |
| `AWD_LoRa_Node5_V6` | `T22D(Lora5)` | 16 min | LoRaTX-Lora5-Setup |

The only differences between the five files are the device ID, the
stagger and the AP name. Do not mix them. The stagger runs once after a
true power-on so five nodes switched on together do not collide; after
that each node adds 0 to 5 random minutes to every sleep.

## Hardware

| Part | Notes |
|---|---|
| ESP32 Dev Module | on the 38-pin expansion board, powered by USB from the boost module |
| Ebyte E220-900T22D | 22 dBm LoRa, UART |
| HC-SR04 | 5 V, echo divided to 3.3 V |
| 15 Ah LiFePO4 + solar | via a 5 V boost module |

### Pins

| Signal | GPIO |
|---|---|
| E220 RX (ESP TX) | 17 |
| E220 TX (ESP RX) | 16 |
| E220 M0 | 21 |
| E220 M1 | 19 |
| E220 AUX | 18 (required, the node waits on it) |
| HC-SR04 TRIG | 13 |
| HC-SR04 ECHO | 1 kΩ to GPIO12, 2 kΩ from GPIO12 to GND |
| Config jumper | 15 to GND |

M0 and M1 must stay on their GPIOs. V6 drives them HIGH before deep
sleep so the E220 sleeps at a few microamps instead of drawing 12 mA in
receive mode all night. If they are soldered to GND that saving is lost.

GPIO12 is a strapping pin. The 1 kΩ / 2 kΩ divider keeps it low at boot;
keep the divider.

## Radio settings

Each wake, while the module is still in sleep/config mode, the node
writes a volatile register block (C2) and only then switches to normal
mode: 9600 8N1, 2.4 kbps, **channel 15 (865.125 MHz)**, 22 dBm,
transparent, RSSI byte off. Nothing is saved in the module, so a swapped
module works without any GUI configuration. `DEVICE_CHANNEL` must equal
the gateway's `LORA_CHANNEL`.

## Frame format

```
T22D(Lora3)|31.52|28.48\r\n
```
device ID, distance from sensor to water in cm, water level in cm. On a
failed reading (fewer than three valid echoes) both numbers are `-1.00`
and the gateway records `SensorError`.

## Config portal

Short GPIO15 to GND, reset. Join the node's AP (password `12345678`),
open `http://192.168.4.1`, login `admin` / `password`.

Fields: **Sleep Interval Base** in seconds (300 to 3600, default 1200)
and **Pipe Height** in cm (default 60). The page shows live distance and
level every 1.5 s, which is the easiest way to check the sensor
mounting. Save restarts the node in normal mode.

Note: nodes Lora4 and Lora5 in the field were set to 600 s and 900 s.
Shorter intervals cost battery and airtime; 1200 s is the intended
value.

## Wake cycle

1. Release the pin holds from deep sleep.
2. Configure the E220 (mode 3), switch to mode 0, wait for AUX.
3. Five ultrasonic pings, 30 ms apart; average of valid readings.
4. Send the frame; wait for AUX to drop and rise again (packet on air
   done).
5. Park the E220 in mode 3, hold M0/M1/TRIG, deep sleep for base plus
   0 to 300 s.

Awake time is about two seconds. Sleep current is dominated by the
DevKit's USB chip and regulator (about 10 mA) and the boost module;
the firmware cannot reduce those.

## Flashing

Board ESP32 Dev Module, 115200 baud. Expect on serial:
```
[SYS] T22D(Lora1) V6 Booting (ch 15)...
[LORA] C2 config OK (9 bytes: C1 00 06 00 00 62 00 0F 03)
[TX] Distance: 31.52cm | Water: 28.48cm
[TX] Payload: T22D(Lora1)|31.52|28.48
[TX] Sleeping 1387s (base=1200 + jitter=187)
```

## Troubleshooting

- `C2 config FAILED`: the module did not answer. Check M0/M1 (both
  HIGH at that moment), the UART cross-wiring, and 3.3 V at the module.
- `AUX never dropped`: AUX not wired to GPIO18, or the module ignored
  the frame. The node still waits 600 ms, so data usually gets through.
- `All readings failed`: sensor not powered (5 V), echo divider broken,
  or the sensor is closer than 2 cm or further than 65 cm from the
  target.
- The node transmits but the gateway never lists it: wrong channel on
  one side, or out of range. Check the gateway's per-node RSSI first.
- Node silent overnight, fine by day: battery or boost module. Some
  power-bank style boost boards switch off when the load drops below
  about 50 mA in deep sleep. Use a plain boost converter.
