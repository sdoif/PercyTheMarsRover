# PercyTheMarsRover

## Aim of the task
> The aim of this project is to design and build an autonomous rover system that could be used in a remote location without direct supervision. The  rover will have a processing unit that is capable to receive movement commands and send status data. In doing this it needs  to  detect  and  avoid obstacles in it's working area. Over time the rover should be able to build a map of its local working area (including those obstacles) on an offsite data store. 
> A charging station will be also designed and implemented to charge the batteries used to power up the rover.

## Subsystems 
This project is broken down into 6 subsystems as indicated by the folder structure : Command, Control, Drive, Energy, Integration, Vision, each operating on a specific section of the Mars Rover.

![Task subsystem, image provided by Imperial's EE Department](https://github.com/sdoif/PercyTheMarsRover/blob/main/subsystems.png?raw=true)

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