# calibrate.py
import numpy as np

# --- Paste your collected data here ---
# The pixel radius values you recorded
pixel_radius_x = [195, 148, 115, 78, 58, 47, 39]
# The real distances you measured with a tape measure
real_distance_y = [30, 40, 50, 75, 100, 125, 150]

# --- Calculate the coefficients ---
# Use a 2nd degree polynomial (quadratic)
# This will calculate the A, B, and C for y = Ax^2 + Bx + C
coefficients = np.polyfit(pixel_radius_x, real_distance_y, 2)

# --- Print the results ---
print("Your new coefficients (A, B, C) are:")
print(coefficients)
print("\nUpdate your main script with these values:")
print(f"CALIB_A = {coefficients[0]}")
print(f"CALIB_B = {coefficients[1]}")
print(f"CALIB_C = {coefficients[2]}")