import math
import json
import sys

DEG_TO_RAD = math.pi/180

# Formats a Java AlignmentVector
def formatVector(name, target, angle, tags, decimals=3):
  return "new AlignmentVector(\"{}\", new Translation2d({}, {}), Rotation2d.fromDegrees({}){}{})".format(
    name,
    round(target[0], decimals),
    round(target[1], decimals),
    round(angle, decimals),
    ", " if len(tags) != 0 else "",
    ", ".join(str(t) for t in tags)
  )

# Offsets a position by a distance and angle (in degrees)
def offsetPosition(input, distance, angle):
  return (
    input[0] + (distance * math.cos(angle * DEG_TO_RAD)),
    input[1] + (distance * math.sin(angle * DEG_TO_RAD))
  )

# Gets tag pose
def getTagPose(tagJson, tagId):
  for t in tagJson["tags"]:
    if t["ID"] == tagId:
      return t["pose"]
    
  print(f"Could not find tag id {tagId}")
  return None

# See AlignmentVector.java
def generate(aprilTagFile, reefCenterOffsets, overallReefOffset):
  with open(aprilTagFile, "r") as f:
    tags = json.load(f)

  # Init output
  leftVectors = []
  centerVectors = []
  rightVectors = []

  # Generate reef vectors
  reefNames = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"]
  reefTags = [18, 17, 22, 21, 20, 19]
  redReefTags = [7, 8, 9, 10, 11, 6]

  for i in range(6):
    pose = getTagPose(tags, reefTags[i])
    pose = (pose["translation"]["x"], pose["translation"]["y"])
    angle = i * 60

    alignmentTags = [reefTags[i], redReefTags[i]]

    # Add offset
    pose = (pose[0] + overallReefOffset[0], pose[1] + overallReefOffset[1])

    # Create center vector
    centerVectors.append(formatVector(f"REEF_{i+1}", pose, angle, alignmentTags))
    
    # Create left vector
    left = offsetPosition(pose, reefCenterOffsets, angle+90)

    leftVectors.append(formatVector(f"REEF_{reefNames[i*2]}", left, angle, alignmentTags))

    # Create right vector
    right = offsetPosition(pose, reefCenterOffsets, angle-90)

    rightVectors.append(formatVector(f"REEF_{reefNames[i*2 + 1]}", right, angle, alignmentTags))

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
    sys.argv[1], # AprilTag .json file
    0.329 / 2.0, # offset of each reef pole from the center of that face of the reef (meters)
    [0, 0.0635] # x/y offset to add to every single position to fix constant error (meters)
  )