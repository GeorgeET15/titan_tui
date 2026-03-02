#!/usr/bin/env python3
import cv2
import numpy as np
import sys
import os

def clean_map(input_path):
    if not os.path.exists(input_path):
        print(f"Error: {input_path} not found")
        return

    # Load image in grayscale
    img = cv2.imread(input_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not decode {input_path}")
        return

    # ROS Map Conventions:
    # 0 (Black)   : Occupied
    # 254/255 (White): Free Space
    # 205 (Gray)  : Unknown
    
    # 1. Salt & Pepper Noise Removal (Median Filter)
    # This removes the "single pixel" artifacts often found in lidar scans
    denoised = cv2.medianBlur(img, 3)

    # 2. Sharpen Walls
    # We want to make walls (0) more solid and free space (255) cleaner.
    # We'll use a morphological 'opening' to remove small free-space noise in walls
    # and 'closing' to fill small holes in the free space.
    
    kernel = np.ones((2,2), np.uint8)
    
    # Remove small specks of occupancy from free space
    # (Morphological Opening on the 'occupied' part)
    # Since occupied is 0, we actually invert logic or work on mask
    mask_occupied = (denoised < 50).astype(np.uint8) * 255
    mask_occupied = cv2.morphologyEx(mask_occupied, cv2.MORPH_OPEN, kernel)
    
    # 3. Reconstruct the map
    # We keep the unknown areas (around 205) as they are, but clean up the rest.
    final = denoised.copy()
    
    # Force free space to be cleaner (anything > 230 becomes 254)
    final[denoised > 230] = 254
    
    # Apply cleaned occupancy mask
    # Everywhere the mask is 0 (after opening), we should probably make it free space
    # if it was originally occupied.
    original_occupied = (denoised < 50)
    cleaned_occupied = (mask_occupied > 128)
    
    # If it was occupied but is no longer in cleaned_occupied, make it free space
    to_clear = original_occupied & ~cleaned_occupied
    final[to_clear] = 254
    
    # Overwrite the original file
    cv2.imwrite(input_path, final)
    print(f"Map {input_path} post-processed and cleaned.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: clean_map.py <path_to_pgm>")
    else:
        clean_map(sys.argv[1])
