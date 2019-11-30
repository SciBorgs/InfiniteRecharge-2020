# Main

This is (effectively) the top level of our code.

There are a few files that are fundamental to how the rest of the code works in this top level, for instance Robot.java and OI.java.

There are also a number of folders or packages. Click on them! They all have READMEs so you can know what they do :D

### Individual file explanations:
- _Robot.java_: essentially the "master" file. It calls every other file, except for some commands, so you can generally trace back most code to beign called somwhere in Robot.java.<enter>
- _Main.jav_: calls Robot.java
- _PortMap.java_: a mapping of sensors and actuators to ports
- _OI.java_: contains logic to run commands based on Joystick and other physical controller input.
- _Utils.java_: general utils file for the entire robot
