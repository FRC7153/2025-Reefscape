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
def generateReefVectors(tags, reefCenterOffsets, overallReefOffset):
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
    #pose = (pose[0] + overallReefOffset[0], pose[1] + overallReefOffset[1])
    pose = offsetPosition(pose, overallReefOffset[0], angle)
    pose = offsetPosition(pose, overallReefOffset[1], angle + 90)

    # Create center vector
    centerVectors.append(formatVector(f"REEF_{i+1}", pose, angle, alignmentTags))
    
    # Create left vector
    left = offsetPosition(pose, reefCenterOffsets, angle+90)

    leftVectors.append(formatVector(f"REEF_{reefNames[i*2]}", left, angle, alignmentTags))

    # Create right vector
    right = offsetPosition(pose, reefCenterOffsets, angle-90)

    rightVectors.append(formatVector(f"REEF_{reefNames[i*2 + 1]}", right, angle, alignmentTags))

  # Output all vectors
  print("REEF LEFT VECTORS:")
  print(",\n".join(leftVectors))

  print("REEF CENTER VECTORS:")
  print(",\n".join(centerVectors))

  print("REEF RIGHT VECTORS:")
  print(",\n".join(rightVectors))

def generateCageVectors(tags, cageCenterOffsets):
  bargeTags = [14, 15, 4, 5]

  centerPose = getTagPose(tags, 14)
  centerPose = (centerPose["translation"]["x"], centerPose["translation"]["y"])

  # Create center cage
  print("CAGE CENTER:")
  print(formatVector("CAGE_CENTER", centerPose, 180, bargeTags))

  # Create left cage
  left = offsetPosition(centerPose, cageCenterOffsets, 90)

  print("CAGE LEFT:")
  print(formatVector("CAGE_LEFT", left, 180, bargeTags))

  # Create right cage
  right = offsetPosition(centerPose, cageCenterOffsets, -90)

  print("CAGE RIGHT:")
  print(formatVector("CAGE_RIGHT", right, 180, bargeTags))

def generateAlgaeVectors(tags):
  processorTags = [16, 3]

  processor = getTagPose(tags, 16)
  processor = (processor["translation"]["x"], processor["translation"]["y"])

  # Create processor vector
  print("PROCESSOR:")
  print(formatVector("PROCESSOR", processor, 270, processorTags))

def generateCoralStationVectors(tags, centerOffset, distanceCenterToLeft, distanceCenterToRight):
  coralStationTags = [13, 12, 2, 1]

  leftVectors = []
  centerVectors = []
  rightVectors = []

  # Create left/right station vectors
  for i, side in enumerate(["LEFT", "RIGHT"]):
    center = getTagPose(tags, coralStationTags[i])
    center = (center["translation"]["x"], center["translation"]["y"])
    angle = 306 - (252 * i) # left is 306 degrees, right is 54 degrees

    center = offsetPosition(center, centerOffset, angle+90)

    # Left side
    leftVectors.append(formatVector(
      f"{side}_CORAL_LEFT",
      offsetPosition(center, distanceCenterToLeft, angle-90),
      angle,
      coralStationTags
    ))

    # Center
    centerVectors.append(formatVector(f"{side}_CORAL_CENTER", center, angle, coralStationTags))

    # Right side
    rightVectors.append(formatVector(
      f"{side}_CORAL_RIGHT",
      offsetPosition(center, distanceCenterToRight, angle+90),
      angle,
      coralStationTags
    ))

  # Output
  print("LEFT CORAL STATION VECTORS (L, R):")
  print(",\n".join(leftVectors))

  print("CENTER CORAL STATION VECTORS (L, R):")
  print(",\n".join(centerVectors))

  print("RIGHT CORAL STATION VECTORS (L, R):")
  print(",\n".join(rightVectors))

# Run
if __name__ == "__main__":
  # Open april tag file
  with open(sys.argv[1], "r") as f:
    tags = json.load(f)

  # Generate reef vectors
  generateReefVectors(
    tags,
    0.329 / 2.0, # offset of each reef pole from the center of that face of the reef (meters)
    [0, 0.0635] # x/y offset to add to every single position to fix constant error (meters)
  )

  # Generate cage vectors
  generateCageVectors(
    tags,
    1.0922 # offset from left/right cage to center cage (meters)
  )

  # Generate algae vectors
  generateAlgaeVectors(tags)

  # Generate coral station vectors
  generateCoralStationVectors(
    tags,
    0.117475, # offset between tag and center position (meters)
    0.61, # offset between center and left position (meters)
    0.406 # offset between center and right position (meters)
  )
