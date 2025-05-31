# Coding challenge

## Introduction
The goal of this challenge was to take the linear velocity $V_d$ and the angular velocity $\omega$ commands for a differential drive robot, and convert them into linear velocity $V_A$ and steering angle $\delta$ commands for an Ackermann drive robot.
The description of the challenge only mentioned "Ackerman", however, in the example figure and variable description, the bicycle model was shown, and is what was implemented here.

## Difficulty
The conversion from differential drive commands to Ackermann commands has inherent limitations due to the fact that differential drive robots have a degree of mobility of 2, while ackerman drive robots have a degree of mobility of 1. 
In fact, the differential drive robot can directly acces the orientation and translation, while the Ackerman drive can only access translation, and indirectly control orientation through steering. 
This is manifested by the fact that the instantaneous center of rotation (ICR) of differential drive robots can be anywhere on the line connecting the two wheels, be it outside of them or between them. However, for Ackerman drives, the ICR can only be "outside" of the chassis, and is further constrained by the limited steering angle of the wheels.
It is therefore difficult to find a good mapping for all cases, but we can try to tailor the approach for special cases.

## "Nominal" case
We will call "nominal" case the case where the ICR is not between the wheels, which can be seen as when the differential drive commands' linear velocity is not too small and the angular velocity is not too high.
Through the kinematics, a relatively direct mapping can be made to the Ackerman commands. For the differential drive, $V_d = \omega \cdot R$, $R$ being the ICR radius. 
For the Ackerman drive, $R = \frac{L}{\tan(\delta)}$, $L$, being the wheelbase, and the other variables as before. Combining these two equations allows to write $\delta = \arctan(\frac{\omega L}{V_d})$. The forward velocity is kept as is $V_A = V_d$

This mapping naturally shines a light on the problem of low forward differential drive velocity, because the velocity is in the denominator of the fraction. However, this fraction is inside an arctangent, which means the angle $\delta$ would just approach $\pm \frac{\pi}{2}$ depending on the sign of $\omega$. 
Even so, this is not possible due to the mechanical contraints of a physical Ackerman drive.

## "Spinning on the spot" case
For completeness, this should rather cover the cases where the ICR is placed between the wheels, but because of time constraints we will say this is only the case when the differential drive is spinning on the spot.
When the differential drive has zero forward velocity but non zero angular velocity, it is spinning on the spot. This behavior is approximated by setting $\delta$ to its maximum value (publishing $\frac{\pi}{2}$) and making the forward velocity be proportional to $\omega$, $V_A = \omega \cdot \frac{L}{\tan(\delta_{max})}$, $\delta_{max}$ being an approximation from what I could find online and on the urdf. The constant that multiplies $\omega$ is the inverse of the maximum path curvature (in m^-1) the Ackermann drive can have. 


## Systemd Service for Automatic Startup

This package includes a `systemd` service file (`config/cmd_converter.service`) and an installation script (`scripts/install_service.sh`) intended to allow the Ackermann converter node to start automatically on system boot.

The `install_service.sh` script is designed for a Linux system where `systemd` is the init process (PID 1). My Docker container did not run `systemd` as PID 1 by default. Therefore, running `sudo ./install_service.sh` in my 
Docker container setup resulted in an error: `System has not been booted with systemd as init system (PID 1). Can't operate.`

I included the `cmd_converter.service` file in the `config` folder and an install script in the `scripts` folder, but could not test them to a full extent myself.

## Setting the package up
You can pull this repo inside a folder in the root of your ros workspace. Assuming the hunter workspace is in the home directory:
```bash
cd ~/hunter_ws/src
git clone https://github.com/lucajg/cmd_conversion.git
```

## Dummy command publisher
My node includes the code for a dummy differential drive command publisher which can be activated or deactivated with the `TEST` boolean.

## Testing
To test the node, I launched it as follows from the terminal after building and launching hunter in gazebo:
```bash
cd ~/hunter_ws
colcon build --symlink-install --packages-select cmd_conversion
source install/setup.bash
ros2 launch cmd_conversion diff_to_ack.launch.py
```

## Challenges encountered
- I had a problem setting up a Docker container from scratch, because there was no GUI so the hunter package would not work (it could not open rviz or gazebo). I ended up using the Docker image from the ROS course I followed this semester, which included the Ubuntu GUI.
- Some things were not very clear in the Hunter documentation.
- I had never implemented a systemd service before, and could not complete this step. The files I attempted to write related to this step are nevertheless available for your review.
