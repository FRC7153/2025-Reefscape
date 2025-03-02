import os
import can
import pigpio
import time
import bitarray
import math

# Init CAN bus
def initCAN() -> can.ThreadSafeBus:
  os.system("sudo ip link set can0 type can bitrate 1000000")
  os.system("sudo ifconfig can0 up")
  bus = can.ThreadSafeBus(channel="can0", bustype="socketcan")
  bus.set_filters(None)
  return bus

# Convert int to binary string
def intToBinary(value: int, length: int) -> str:
  return "{0:b}".format(value).zfill(length)[:length]

# Format CAN arbitration id
def formatArb(devType: int, manCode: int, apiClass: int, apiIndex: int, deviceId: int) -> int:
  arb = intToBinary(devType, 5)
  arb += intToBinary(manCode, 8)
  arb += intToBinary(apiClass, 6)
  arb += intToBinary(apiIndex, 4)
  arb += intToBinary(deviceId, 6)
  return int(arb, 2)

# Send a message on the CAN bus
def sendMessage(bus: can.ThreadSafeBus, arb: int, data: int) -> None:
  msg = can.Message(
    arbitration_id = arb,
    data = data,
    is_extended_id=True
  )

  try:
    bus.send(msg)
  except Exception as e:
    print(f"Failed to send message: {e}")

# PWM port callback
highTick = 0
lowTick = 0
pulseWidth = 0

def pwmCallback(_, level: int, tick: int) -> None:
  global highTick, lowTick, pulseWidth
  if level == 1:
    highTick = tick
  elif level == 0:
    lowTick = tick
    pulseWidth = pigpio.tickDiff(highTick, lowTick)

# Run
if __name__ == "__main__":
  # Config
  CAN_ID = 20
  ENC_IN = 22
  ENC_OFFSET = (0.776622 + .25)
  LIMIT_SWITCH_IN = 23

  ENC_OFFSET = 0

  # Init Pi inputs
  os.system("sudo pigpiod")

  time.sleep(3)

  pi = pigpio.pi()
  #bus = initCAN()
  arb = formatArb(11, 8, 1, 0, CAN_ID) # API 1, index 0

  callback = pi.callback(ENC_IN, pigpio.EITHER_EDGE, pwmCallback)

  while True:
    # Read angle
    angle = (pulseWidth / 1025.0) * 360.0
    angle = ((angle - ENC_OFFSET) * -1) % 360
    #angle = math.floor(angle * 100)
    print(angle)
    time.sleep(0.5)
    continue

    # Read limit switch
    ls = pi.read(LIMIT_SWITCH_IN)

    # Send packet
    packet = bitarray.bitarray()
    packet.extend(intToBinary(angle, 16))
    packet.append(bool(ls))
    packet = packet.zfill(64)

    sendMessage(bus, arb, packet)

    # Wait
    time.sleep(0.02)