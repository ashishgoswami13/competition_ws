#!/usr/bin/env python3
"""
Create a rectangular path texture for line following
Rectangle size: 20cm x 30cm (200mm x 300mm)
Line width: 2cm (20mm)
Ground plane: 1m x 1m texture
"""

import cv2
import numpy as np

# Texture resolution (higher = better quality)
# 1m = 1000mm, so 1 pixel = 1mm for easy calculation
texture_size = 1000  # 1000x1000 pixels = 1m x 1m

# Create white background
img = np.ones((texture_size, texture_size, 3), dtype=np.uint8) * 255

# Rectangle parameters (in mm, converted to pixels)
rect_width = 300  # 30cm
rect_height = 200  # 20cm
line_width = 20   # 2cm thick line

# Center the rectangle
center_x = texture_size // 2
center_y = texture_size // 2

# Calculate rectangle corners (outer edge)
left = center_x - rect_width // 2
right = center_x + rect_width // 2
top = center_y - rect_height // 2
bottom = center_y + rect_height // 2

# Draw the rectangle as 4 thick lines (black)
black = (0, 0, 0)

# Top horizontal line
cv2.rectangle(img, 
              (left, top), 
              (right, top + line_width), 
              black, -1)

# Bottom horizontal line
cv2.rectangle(img, 
              (left, bottom - line_width), 
              (right, bottom), 
              black, -1)

# Left vertical line
cv2.rectangle(img, 
              (left, top), 
              (left + line_width, bottom), 
              black, -1)

# Right vertical line
cv2.rectangle(img, 
              (right - line_width, top), 
              (right, bottom), 
              black, -1)

# Add a starting point marker (small circle)
start_x = center_x - rect_width // 2 + line_width // 2
start_y = center_y
cv2.circle(img, (start_x, start_y), 5, (255, 0, 0), -1)  # Blue start marker

# Save the image
output_path = 'rectangle_20x30.png'
cv2.imwrite(output_path, img)

print(f"✅ Rectangle path created: {output_path}")
print(f"   Texture size: {texture_size}x{texture_size} pixels (1m x 1m)")
print(f"   Rectangle: {rect_width}mm x {rect_height}mm (30cm x 20cm)")
print(f"   Line width: {line_width}mm (2cm)")
print(f"   Center: ({center_x}, {center_y})")
print(f"   Corners: ({left}, {top}) to ({right}, {bottom})")
