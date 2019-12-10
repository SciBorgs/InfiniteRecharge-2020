# RobotState

This folder takes care of everything revolving the RobotState.java, our object which stores all of the data about the robot (also called a deterministic state machine).

It stores lots of information from the angle of the wheels to the position of the robot.

There is also a RobotStateHistory object, which stores a set of RobotStates corresponding to different times.

This is useful for a few reasons. For one, it allows us to perform calculations & predictions more easily. For instance the particle filter (in state estimation) would not really work without a deterministic state machine. Also it will ultimately make logging easier.
