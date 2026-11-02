# myCobot 280 M5 joint-mapping plan (candidate, not hardware-verified)

Vendor Humble URDF arm order:

1. `joint2_to_joint1`
2. `joint3_to_joint2`
3. `joint4_to_joint3`
4. `joint5_to_joint4`
5. `joint6_to_joint5`
6. `joint6output_to_joint6`

Candidate VisionPick mapping is `pymycobot.get_angles()[0..5]` in the same order. This is **not yet approved**.

Gate Review A must verify the mapping on real hardware at several non-zero poses. For each test pose:

- Read `get_angles()` from the physical myCobot 280 M5.
- Load the same six values into the MoveIt state using the candidate mapping.
- Compare the RViz pose against the physical robot, especially shoulder, elbow, wrist bend and wrist rotation.
- Verify signs and zero offsets, not only joint order.
- Record the evidence in the regression/measurement report.

Do not set `planning/robot_model/robot_model_manifest.yaml` to `verified: true` until this test passes.
