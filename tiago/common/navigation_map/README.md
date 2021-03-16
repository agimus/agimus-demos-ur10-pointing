# Calibration of the MoCap body in the mobile base frame.

- save MoCap motions while the robot base follow a constant velocity command.
  I used the following (linear, angular velocities) : (0, 0.3), (0.1, 0.3) and (0.1, -0.3)
  I think using (0.1, 0) would help too.
- export these motion to TSV files.
- tune `calibration_base_to_mocap_frame.py` to read your files.

# Calibration of the MoCap frame in the navigation map

1. Start script `acquire_map_to_mocap_frame.py`.
2. Move the robot around in the MoCap area.
3. Save measurements only when the robot is static.
4. Go to 2. until you have enough measurements (> 6 ?)
