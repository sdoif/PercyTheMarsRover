# PercyTheMarsRover

This repository contains the files required to operate each subsystem of Group 5's end of 2nd year group project titled PercyTheMarsRover. The software operates with the rover hardware provided by Imperial College London's EE department.

This project is broken down into 6 subsystems as indicated by the folder structure : Command, Control, Drive, Energy, Integration, Vision, each operating on a specific section of the Mars Rover. These subsystems operate together to form a rover capable of autonomous and manual motion with object detection where the target objects are 5 different coloured ping pong balls. 

![Task subsystem, image provided by Imperial's EE Department](https://github.com/sdoif/PercyTheMarsRover/blob/main/subsystems.png?raw=true)

Regarding movement of the rover, [Selin or Cem write about how the movement is programmed using the Arduino or whatever the components are]

Using a Terasic D8M-Camera mounted on a DE10-Lite board containing an FPGA, the rover is able to detect the specific ping pong balls, process the frame data and transfer it to other subsystems using an ESP32. The information is then ... [Salman please write about how the information is transferred within the server]

The rover is powered using [Batu pls write about how it was originally going to be powered]

## Autonomous
By processing the frame information from the camera, we are able to measure the distance of objects from the rover. This functionality is fundamental to the autonomous process which follows a 3-stage scanning system to map out all ping pong balls in an area.

#### Stage 1
The rover turns 360° and maps any balls within the distance range of 25 to 80cm. Within this range, we can confirm that the distance measured is accurate and so can plot them on the live mapping on the web app.

#### Stage 2
The rover completes another 360° logging balls within the range 80 to 180cm. At the end of the scan, the rover looks the for the closest ball that was within this range (if there was something logged to be in this distance) and goes to it until it is within a close enough range to measure it's distance accurately (less than 80cm). Once the distance has been measured accuretely, that ball is mapped and the rover repeats the 1st scan.

However, if no balls were logged to be in this range, we advance to the 3rd stage.

#### Stage 3
The rover begins rotating in position until it finds a ball beyond the 180cm mark and then advances towards it. Once within the 80cm range from the ball, we measure the distance and map that ball then repeat the 1st scan.

Once all 5 balls have been mapped, the autonomous scanning process stops.

## Manual
Using the web browser, a user is able to direct the motion of the rover simplying by pressing buttons to advance the rover in different directions.

## Tasks of each subsystem
### Energy
Powers the rover using batteries charged by solar panels.

### Drive
Controls the rover movement including : speed, direction and turning.

### Vision
Uses a camera to detect ping pong balls, apply a filtering and measure their distance from the camera.

### Control
Forms the communication with all subsystems using an ESP32.

### Command
Creates a web browser allowing for remote control of the rover and receives information from the separate subsystems.

### Integration
Connects all subsystems together and ensures everything works together with correct functionality.