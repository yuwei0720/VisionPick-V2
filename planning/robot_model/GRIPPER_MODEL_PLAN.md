# myCobot 280 M5 + official Adaptive Gripper model plan

Confirmed hardware facts:

- Robot: Elephant Robotics myCobot 280 M5.
- End effector: official Elephant Robotics myCobot Adaptive Gripper.
- RealSense: external workcell-mounted sensor, approximately 480 mm above the worktable; it is **not** a robot-mounted link.

## Candidate model source

Use the Elephant Robotics `mycobot_ros2` Humble description advertised as `mycobot 280 m5 adaptive gripper` as the vendor baseline. The selected URDF/xacro and every referenced mesh must be snapshotted, hashed, and reviewed on the Ubuntu/ROS 2 host before hardware verification.

The vendor product page lists an overall size of 112 x 94 x 50 mm, 100 g mass, and a 20–45 mm stroke. These values are sanity checks only; they are not substituted for link-level collision geometry.

## Collision states

The model must represent at least:

1. OPEN / maximum conservative finger envelope.
2. CLOSED / minimum opening state.

Until movable finger-joint semantics are verified, Guard calculations should prefer a conservative envelope rather than under-estimating finger reach.

## Critical self-collision pairs

Do not disable these pairs without physical evidence:

- gripper ↔ wrist
- gripper ↔ forearm
- gripper ↔ upper arm
- gripper ↔ base

The near-base top-down collision case is the primary regression scenario.

## External RealSense

Because the RealSense is fixed to the workcell rather than the robot:

- do not attach it to `joint6_flange` or any robot link;
- record it in the Planning Scene only if its body/support geometry can intersect the robot workspace;
- do not use the approximate 480 mm height as a calibrated collision pose;
- measure/calibrate exact XYZ/RPY in `robot_base` before marking the scene verified.
