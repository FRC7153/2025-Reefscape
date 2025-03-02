# RaspberryPi
We used a RaspberryPi 4b on the elevator of our robot to process inputs from a REV Through Bore Encoder and a limit switch. It communicated with the RoboRio via CAN bus through a [RS485 CAN Hat](https://www.waveshare.com/rs485-can-hat.htm).

## CAN API
See [FRC CAN Specs](https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html) for more info.

#### Refresh (Class 1, Index 0)
Standard refresh packets sent from the RaspberryPi periodically, containing all the data the robot needs.

Data (8 bytes, 64 bits):
| 0 - 15 | 16 | 17 - 63 | 26 | 27 | 28 | 29 - 35 | 36 - 42 | 43 - 49 | 50 - 55 | 56 - 63 |
|---|---|---|---|---|---|---|---|---|---|---|
|Arm angle (deg * 1000)|Limit Switch Status|Distance (mm)|Empty|

## Startup
To start the python file automatically, the following line was added in the */etc/rc.local* file:
```
## Run 2023 code:
sudo python3 /home/pi/SeasonCode/2025-Reefscape/elevator-pi/main.py
```

## Libraries
- [python-can](https://pypi.org/project/python-can/)
- [bitarray](https://pypi.org/project/bitarray/)