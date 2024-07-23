# RPFC-Drone (RPi Pico Flight Controller)

## A simple to make Flight Controller based on the RPi Pico and the MPU6500 IMU Module.

### Features:
1. 400Hz Refresh Rate
2. Low Cost
3. Low Battery Warning
4. Throttle Stick warning

Note: The current version does not support the Auto-Level feature

### Steps to Make:
1. Create the Schematic provided on a PCB (Refer the Image below)

<p align="center"><img src="https://github.com/devashishlahariya9/RPFC-Drone/blob/main/Images/RPFC-Drone Schematic.png" width="550" height="400"> </p>

2. Build the code or directly upload the .uf2 file in "Code/build/" folder.
3. Securely mount the Flight Controller inside Drone Body (Refer to the images below)

<p align="center">
  <img src="https://github.com/devashishlahariya9/RPFC-Drone/blob/main/Images/RPFC_Top.jpg" width="235" height="400">
  <img src="https://github.com/devashishlahariya9/RPFC-Drone/blob/main/Images/Drone_Orientation.jpg" width="400" height="400">
</p>

4. Calibrate the ESCs.
5. Connect a 3S LiPo 30/40C Battery and wait till only Green LED is ON.
6. Arm the Drone by flipping the Channel 5 Switch with its Potentiometer set to lowest value.
7. Enjoy!

**NOTE: DO NOT MOVE OR TOUCH THE DRONE DURING THE IMU CALIBRATION PROCEDURE THAT IS DONE 2 SECONDS AFTER CONNECTING THE BATTERY.**

### LED Indication Codes:
#### Drone Unarmed:
1. Blue LED ON After Connecting Battery: IMU Calbration under progress.
2. Blue LED OFF and Green LED ON: IMU Calibration Complete, Drone Ready to Fly.
3. Blue LED Blinking with delay of 250ms: Throttle Stick not in Lowest Position Warning.
4. Blue LED Blinking with delay of 1sec: Battery LOW Warning, it is not possible to fly the Drone with this error. 

#### Drone Armed:
1. Blue LED turns ON during Flight: Battery LOW Warning.