# FSMBotTemplate

A Java 17 WPILib robot template with a manually updated finite-state-machine drivetrain, Xbox teleop, and optional Limelight vision. The intended robot-specific hardware integration is a single unmodified Phoenix Tuner X-generated `TunerConstants.java` file.

## Phoenix Tuner X integration

Replace `src/main/java/frc/robot/generated/TunerConstants.java` with this robot's **2026 Phoenix Tuner X Swerve Project Generator** output. The generated class must retain the `frc.robot.generated` package and its normal `createDrivetrain()` factory, and import the template's `frc.robot.subsystems.CommandSwerveDrivetrain`. Keep the template wrapper; do not copy Tuner X's generated wrapper file over it or edit the generated `TunerConstants.java` afterward.

That generated file supplies the hardware-specific CAN bus, device constructors and IDs, module positions, encoder offsets, motor gains, and measured `kSpeedAt12Volts`. The reusable wrapper and FSM provide open-loop-voltage field-centric control, alliance operator perspective, Phoenix pose-estimator time conversion, and Phoenix desktop simulation. No robot IDs, offsets, dimensions, or gains should be duplicated in the template `Constants.java`.

The repository's `TunerConstants.java` is a **compile-time scaffold only**: it has no hardware configuration, and its factory throws intentionally. Replace it before deploying or expecting swerve simulation. This template's integration contract targets the current GradleRIO 2026.2.1 / Phoenix 6 26.1.0 project and standard unmodified 2026 Tuner X output. Verify compatibility against the exact output file your generator creates before your first robot test.

## Xbox controls

Connect one Xbox controller to Driver Station USB port 0. Control mapping:

- Left stick Y: forward/backward.
- Left stick X: left/right strafe.
- Right stick X: counter-clockwise/clockwise rotation.
- Start button: reset field-centric forward on press. Point the robot toward the intended driver-forward direction before pressing Start.

Stick response is linear after an independently applied per-axis deadband. A disconnected controller commands stop; driving resumes only after reconnection and valid stick input. There is no robot-centric toggle, precision mode, slew/ramp limiter, or X-lock binding.

In `Constants.RobotIntegration`:

- `DRIVER_CONTROLLER_PORT` selects the Driver Station port (default `0`).
- `DRIVER_DEADBAND` is the normalized stick deadband (default `0.08`).
- `MAX_LINEAR_SPEED_SCALE` scales the Phoenix Tuner X measured free speed; `1.0` uses the full generated speed and lower positive values provide a cap. Maximum angular speed is derived from the scaled linear limit divided by the generated maximum module radius.

Field-centric controls use operator perspective. Blue forward is assumed until alliance data is available. Alliance perspective updates only while disabled, so late alliance data cannot unexpectedly flip controls during an enabled period. Teleop start does not automatically reset the gyro/heading.

## Robot modes

Teleop uses the Xbox mapping above. Disabled, autonomous, and test modes command zero drive demand. Autonomous path following and autonomous mode selection are not included. The existing FSM and explicit `TimedRobot` mode-loop architecture remains in use; this template does not use WPILib's command scheduler for drivetrain control.

## Optional Limelight vision

Vision support remains available in `VisionFSMSystem`, but is disabled by default. Set `Constants.RobotIntegration.ENABLE_LIMELIGHT_VISION` to `true` only when a Limelight is configured. When false, the robot does not construct the vision FSM or write camera pipeline, LED, camera pose, or MegaTag2 orientation values to NetworkTables; a camera is not required to initialize or drive.

When enabled, default camera name and AprilTag pipeline are in `Constants.RobotIntegration`. `VisionFSMSystem.Config` also supports an optional camera-to-robot transform, field bounds, quality thresholds, pose uncertainty, and coordinate transform. The default camera transform assumes it has already been configured in Limelight Web UI. Valid MegaTag2 measurements are checked and fused into Phoenix's estimator. `imported/LimelightHelpers` is a focused NetworkTables adapter, not the full upstream helper library. Limelight detections are not synthesized in simulation.

## Simulation and dashboard

The existing GradleRIO Sim GUI/Driver Station and Phoenix swerve simulation are used together; the template does not create a second physics model. WPILib `Field2d` named `Field` displays the Phoenix estimated pose. No extra driver/status dashboard is included.

## Before driving on carpet

1. Confirm the generated Tuner file is the 2026 output for this project's Phoenix version, uses the template wrapper import, and has not been locally edited.
2. In Phoenix Tuner, verify the CAN bus and IDs, module order and positions, motor inversions, encoder offsets, and measured free speed. Check that generated module geometry is finite and nonzero.
3. Securely raise the robot so no wheel can propel it. Verify safe neutral behavior, the disconnected-controller stop and reconnection, steering, each module's direction, and the Xbox forward/strafe/rotation signs.
4. At low speed, verify blue and red field-centric forward/strafe behavior. Confirm Start resets field heading only when intentionally pressed and while the robot is pointed at the intended driver-forward direction.
5. Verify disabled, autonomous, and test modes stop the drivetrain. Resolve any initialization diagnostics before attempting floor operation.
6. After elevated checks pass, perform the first floor test at reduced `MAX_LINEAR_SPEED_SCALE` with a clear area and a person ready to disable the robot.

## Build and simulation commands

- `./gradlew simulateJava` — desktop simulation (requires a valid generated Tuner file for simulated drivetrain operation).
- `./gradlew test build` — test and build (Java 17).
- Deploy only after replacing the placeholder `TunerConstants.java` with the correct robot-specific generated file and completing the physical checks above.
