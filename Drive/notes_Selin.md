Optical flow sensor: increase frame rate to maximum(6469)
--> use mousecam_write_reg to write to extended_config to enable writing 
--> mousecam_write_reg to upper and lower bounds

		  : Sensor wrap around
--> Detect every time it wraps around and over flows and set it back to zero after it wraps around
--> Use counter for the 360 turn
--> Quarter counters for the 90 turn
