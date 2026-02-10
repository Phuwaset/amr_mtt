import cv2
import numpy as np
import sys

def main():
    try:
        # Define marker ID and size
        marker_id = 0
        marker_size = 200 # pixels
        
        # Load dictionary
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Generate marker
        marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
        
        # Add white padding to make it visible against dark backgrounds
        pad = 20
        padded_img = np.ones((marker_size + 2*pad, marker_size + 2*pad), dtype=np.uint8) * 255
        padded_img[pad:-pad, pad:-pad] = marker_img
        
        # Save
        cv2.imwrite('aruco_marker_0.png', padded_img)
        print("Generated aruco_marker_0.png with padding")
        
    except AttributeError:
        print("cv2.aruco not found, generating fallback marker")
        # Fallback: Generate a simple pattern manually (ID 0 in 4x4)
        # ID 0 in 4x4 (DICT_4X4_50):
        # 1 0 1 1
        # 0 1 0 1
        # 0 0 1 1
        # 0 0 1 1
        # (This is just an example pattern, roughly correct for test)
        # Actually, let's just make a distinct black/white pattern
        img = np.zeros((200, 200), dtype=np.uint8)
        img.fill(255) # White background
        img[50:150, 50:150] = 0 # Black box
        img[70:90, 70:90] = 255 # White inner
        cv2.imwrite('aruco_marker_0.png', img)

if __name__ == "__main__":
    main()
