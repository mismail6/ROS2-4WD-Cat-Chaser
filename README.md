Note that this project is made for Ros2 Kilted + Gazebo Ionic. It might not work with other versions.


This script will build the package and launch everything:
```bash
./run_all.sh
```

Then in second terminal the camera_view node can be launched to chase the cat:
```bash
ros2 run cat_chaser camera_view
# you can either manually position the existing cat model in the world or use your own 3d cat model
```

The robot design is a simple four-wheel drive robot with the Ackerman Steering plugin.
The following link contains the documentation for the plugin that was helpful to set the relevant parameters:
https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html

The default teleop_twist_keyboard node can be used, but I made a custom one (teleop_keyboard_hold.py) because I wanted to operate the robot like a joystick rather than toggling the commands on a keyboard.
