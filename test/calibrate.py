# calibrate.py
import numpy as np

# The pixel radius values you recorded
pixel_radius_x = [300, 255, 190, 170, 148, 125, 109, 89, 82, 73]
# The real distances you measured with a tape measure
real_distance_y = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65]

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