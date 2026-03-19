import cv2
import numpy as np

# Create an AprilTag 36h11 marker, ID 0
# 36h11 implies 36 bits of data, Hamming distance 11
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

# Generate marker of size 400x400 pixels
marker_img = cv2.aruco.generateImageMarker(dictionary, 0, 400)

# Save to file
output_path = "apriltag_36h11_0.png"
cv2.imwrite(output_path, marker_img)
print(f"Generated {output_path}")
