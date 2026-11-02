# External RealSense planning-scene measurement plan

Confirmed: the RealSense is fixed above the external worktable at approximately 480 mm height.

Before adding it as a verified collision object, record:

- exact RealSense model;
- camera body dimensions or conservative bounding box;
- support/bracket/rail geometry that can enter the robot workspace;
- camera optical/body pose relative to `robot_base` (XYZ and RPY);
- uncertainty/tolerance used for collision padding.

The approximate 480 mm height is useful for inspection but is not sufficient for collision authorization.

If later measurements prove the camera and support are entirely outside every reachable robot configuration with required padding, document that result and it may be omitted from the active Planning Scene while remaining in the workcell manifest.
