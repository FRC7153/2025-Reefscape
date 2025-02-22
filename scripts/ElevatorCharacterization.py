import pandas as pd
import numpy as np
import sys
from sklearn.metrics import r2_score

# Characterize an elevator mechanism from SysId tests. Load .csv file containing state, time,
# voltage, position, velocity, and optionally acceleration.
def characterize(filename, stateColumn, timeColumn, voltageColumn, positionColumn, velocityColumn, accelerationColumn=None):
  # Open file
  data = pd.read_csv(filename)
  print(f"Read {filename}")

  # Drop data not from any of the SysId tests
  states = data[stateColumn]
  originalLen = len(data)

  data = data[states.isin(["quasistatic-forward", "quasistatic-reverse", "dynamic-forward", "dynamic-reverse"])]
  print(f"Using {len(data)} of {originalLen} entries")

  # Get columns
  time = data[timeColumn] # seconds
  voltage = data[voltageColumn] # volts
  position = data[positionColumn] # rotations
  velocity = data[velocityColumn] # rotations/second

  if accelerationColumn in data.columns:
    accel = data[accelerationColumn] # rotations/second^2
  else:
    # Acceleration column missing, compute it:
    accel = np.gradient(velocity, time) # rotations/second^2
    print("Manually computed acceleration. Consider including it in the .csv file.")
  
  # Fit to V = Kg + Ks * sgn(velocity) + Kv * velocity + Ka * acceleration
  # See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#elevator-feedforward 
  
  sgn = np.sign(velocity) # sign of velocity
  X = np.column_stack([np.ones_like(velocity), sgn, velocity, accel])

  params, _, _, _ = np.linalg.lstsq(X, voltage)
  Kg, Ks, Kv, Ka = params

  # Score with and without acceleration
  r2WithAccel = r2_score(voltage, X @ [Kg, Ks, Kv, Ka])
  r2WithoutAccel = r2_score(voltage, X @ [Kg, Ks, Kv, 0.0])

  # Output values
  print(f"kG = {Kg} (output)")
  print(f"kS = {Ks} (output)")
  print(f"kV = {Kv} (output/rps)")
  print(f"kA = {Ka} (output/(rps/second))")
  print(f"r^2 = {r2WithAccel} (with acceleration feedforward)")
  print(f"r^2 = {r2WithoutAccel} (without acceleration feedforward)")

# Run
if __name__ == "__main__":
  characterize(
    sys.argv[1], # .csv file name
    "Elevator-SysID-State", # SysId state column name
    "Timestamp", # Timestamp (seconds) column name
    "Phoenix6/TalonFX-14/MotorVoltage", # Motor voltage (volts) column name
    "Phoenix6/TalonFX-14/Position", # Motor position (rotations) column name
    "Phoenix6/TalonFX-14/Velocity", # Motor velocity (rotations/second) column name
    "Phoenix6/TalonFX-14/Acceleration" # Motor acceleration (rotations/second^2) column name, 
    # or None if it was not recorded.
  )
