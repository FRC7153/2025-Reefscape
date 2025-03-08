import math

DEG_TO_RAD = math.pi/180

# Formats a Java AlignmentVector
def formatVector(name, target, angle, decimals=3):
  return "new AlignmentVector(\"{}\", new Translation2d({}, {}), Rotation2d.fromDegrees({}))".format(
    name,
    round(target[0], decimals),
    round(target[1], decimals),
    round(angle, decimals)
  )

# Offsets a position by a distance and angle (in degrees)
def offsetPosition(input, distance, angle):
  return (
    input[0] + (distance * math.cos(angle * DEG_TO_RAD)),
    input[1] + (distance * math.sin(angle * DEG_TO_RAD))
  )

# See AlignmentVector.java
def generate(reefCenterOffsets):
  # Each reef position (from the corresponding AprilTag position, from the AndyMark field)
  reef = [(3.66, 4.02), (4.07, 3.30), (4.90, 3.30), (5.32, 4.02), (4.90, 4.74), (4.07, 4.74)]
  reefNames = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]

  # Init output
  leftVectors = []
  centerVectors = []
  rightVectors = []

  # Generate reef vectors
  for i in range(6):
    angle = i * 60

    # Create center vector
    centerVectors.append(formatVector(f"REEF_{i+1}", reef[i], angle))
    
    # Create left vector
    left = offsetPosition(reef[i], reefCenterOffsets, angle+90)

    leftVectors.append(formatVector(f"REEF_{reefNames[i*2]}", left, angle))

    # Create right vector
    right = offsetPosition(reef[i], reefCenterOffsets, angle-90)

    rightVectors.append(formatVector(f"REEF_{reefNames[i*2 + 1]}", right, angle))

  # TODO generate intake vectors

  # Output all vectors
  print("LEFT VECTORS:")
  print(",\n".join(leftVectors))

  print("CENTER VECTORS:")
  print(",\n".join(centerVectors))

  print("RIGHT VECTORS:")
  print(",\n".join(rightVectors))

# Run
if __name__ == "__main__":
  generate(
    0.1635125 # offset of each reef pole from the center of that face of the reef (meters)
          # Currently half of 12 7/8"
  )