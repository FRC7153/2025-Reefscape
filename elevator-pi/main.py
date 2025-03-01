import os
import can
import pigpio
import time
import bitarray
import math

def initCAN() -> can.ThreadSafeBus:
  os.system("sudo ip link set can0 type can bitrate 1000000")
  os.system("sudo ifconfig can0 up")
  bus = can.ThreadSafeBus(channel="can0", bustype="socketcan")
  bus.set_filters(None)
  return bus

def intToBinary(value: int, length: int) -> str:
  return "{0:b}".format(value).zfill(length)[:length]

def formatArb(devType: int, manCode: int, apiClass: int, apiIndex: int, deviceId: int) -> int:
  arb = intToBinary(devType, 5)
  arb += intToBinary(manCode, 8)
  arb += intToBinary(apiClass, 6)
  arb += intToBinary(apiIndex, 4)
  arb += intToBinary(deviceId, 6)
  return int(arb, 2)

def sendMessage(bus: can.ThreadSafeBus, arb: int, data: int):
  msg = can.Message(
    arbitration_id = arb,
    data = data,
    is_extended_id=True
  )

  try:
    bus.send(msg)
  except Exception as e:
    print(f"Failed to send message: {e}")

# Run
if __name__ == "__main__":
  os.system("sudo pigpiod")
  pi = pigpio.pi()
  bus = initCAN()

  # Init PWM Absolute encoder
  ENC_IN = 22
  ENC_OFFSET = (0.776622 + .25)
  LS_IN = 23

  while True:
    angle = (pi.get_servo_pulsewidth(ENC_IN) / 1025.0) * 360.0
    angle = ((angle - ENC_OFFSET) * -1) % 360
    angle = math.floor(angle * 100)

    ls = pi.read(LS_IN)

    packet = bitarray.bitarray()
    packet.extend(intToBinary(angle, 16))
    packet.append(bool(ls))
    packet = packet.zfill(64)

    sendMessage(
      bus,
      formatArb(11, 8, 1, 0, 20),
      packet
    )

    time.sleep(0.015)