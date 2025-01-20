import numpy as np
import json
import sys

# Convert 3d rotation matrix to quaternion
def rotationMatrixToQuaternion(R):
  trace = np.trace(R)
  if trace > 0:
    S = 2 * np.sqrt(trace + 1.0)
    q_w = 0.25 * S
    q_x = (R[2, 1] - R[1, 2]) / S
    q_y = (R[0, 2] - R[2, 0]) / S
    q_z = (R[1, 0] - R[0, 1]) / S
  else:
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
      S = 2 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
      q_w = (R[2, 1] - R[1, 2]) / S
      q_x = 0.25 * S
      q_y = (R[0, 1] + R[1, 0]) / S
      q_z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
      S = 2 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
      q_w = (R[0, 2] - R[2, 0]) / S
      q_x = (R[0, 1] + R[1, 0]) / S
      q_y = 0.25 * S
      q_z = (R[1, 2] + R[2, 1]) / S
    else:
      S = 2 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
      q_w = (R[1, 0] - R[0, 1]) / S
      q_x = (R[0, 2] + R[2, 0]) / S
      q_y = (R[1, 2] + R[2, 1]) / S
      q_z = 0.25 * S

  quaternion = np.array([q_w, q_x, q_y, q_z])
  return quaternion / np.linalg.norm(quaternion)

def convertFMAP(path, outputPath="AprilTagLayout.json"):
  tags = []

  # Load file
  file = open(path, "r")
  fmap = json.load(file)
  print(f"Loaded {path}")

  # Process tags
  for tag in fmap["fiducials"]:
    t = tag["transform"]

    # Get rotation
    quat = rotationMatrixToQuaternion(
      np.array([
        [t[0], t[1], t[2]],
        [t[4], t[5], t[6]],
        [t[8], t[9], t[10]]
      ])
    )

    # Format fmap entry
    tags.append({
      "ID": tag["id"],
      "pose": {
        "translation": {
          "x": t[3],
          "y": t[7],
          "z": t[11]
        },
        "rotation": {
          "quaternion": {
            "W": quat[0],
            "X": quat[1],
            "Y": quat[2],
            "Z": quat[3]
          }
        }
      }
    })

    print(f"Formatted tag {tag["id"]}")

  # Format output
  output = {
    "tags": tags,
    "field": {
      "length": fmap["fieldlength"],
      "width": fmap["fieldwidth"]
    }
  }

  # Save file
  with open(outputPath, "w+") as outputFile:
    json.dump(output, outputFile, indent=2)
    print(f"Saved to {outputPath}")

  file.close()

# Run
if __name__ == "__main__":
  path = sys.argv[1]
  convertFMAP(path)
