# Vision

This directory handles the DE10-Lite board and it's camera module, the Terasic D8M-GPIO. Here, we configure the FPGA to process the frames captured by the camera and analyse them using a NIOS II processor instantiated onto the FPGA.

The Vision submodule is split into two core files : EEEIMGPROC.v and main.c, each operating on the two main components of the FPGA.

## EEEIMGPROC.v
This module is configured on the FPGA and:
- analyses each pixel in the frames captured by the D8M-GPIO camera,
- applies a Gaussian blur,
- calculates the HSV values from the RGB values,
- identifies what colour the pixel is based on the HSV values,
- replaces background pixels with black and highlights pixels on the ping pong balls,
- applies a custom scan that looks at the previous 6 pixels and replaces the current pixel with black if 2 or more of the previous pixels are black,
- creates a boundary box around each ping pong ball based on the positions of that ping pong ball's specific colour
- uses a state machine to send the boundary box coordinates and their corresponding colour to the NIOS II processor.

## main.c
This c file runs on the NIOS II processor and does some post-processing to the boundary box coordinates acquired from the camera frames.

The main implementations:
- Measures the dimensions of the boundary boxes formed around the ping pong balls,
- Determines if the height and width of the box are similar enough to consider this box as that of a ping pong ball,
- Measures the central pixel of the boundary box and determines if camera is facing the ping pong ball head on
- Measures the distance of the ping pong ball from the camera
- Keeps track of what balls have already been seen overall and the closest ball in the 2nd scan
- Uses a closed-loop system to ensure that the rover stays on track when going towards a ball
- Forwards the necessary information to Control using UART.