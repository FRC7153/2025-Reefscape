import math
import json
import sys

DEG_TO_RAD = math.pi/180

# Formats a Java Translation2d
def formatTranslation(target, decimals=3):
  return "new Translation2d({}, {})".format(
    round(target[0], decimals),
    round(target[1], decimals)
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

# See LockOnReefCommand.java
def generate(aprilTagFile, extension=15):
  with open(aprilTagFile, "r") as f:
    tags = json.load(f)

  # Init output
  center = None # average of all tags
  zoneEdges = []

  # Generate reef vectors
  reefTag = [18, 17, 22, 21, 20, 19]

  # Calculate center
  for i in range(6):
    pose = getTagPose(tags, reefTag[i])
    pose = [pose["translation"]["x"], pose["translation"]["y"]]

    if center is None:
      center = pose
    else:
      center[0] = (center[0] + pose[0]) / 2
      center[1] = (center[1] + pose[1]) / 2

  # Calculate zone edges
  for i in range(6):
    angle = i * 60
    zoneEdges.append(formatTranslation(offsetPosition(center, extension, angle + 150)))

  # Output all positions
  print("CENTER:")
  print(formatTranslation(center))

  print("EDGES:")
  print(",\n".join(zoneEdges))

# Run
if __name__ == "__main__":
  generate(
    sys.argv[1], # AprilTag .json file
  )