# Xbox Swerve Template Specification

**Status:** Interviewed requirements; implementation not started from this interview request.  
**Purpose:** Define the next implementation of the FRC swerve template so a robot integration only supplies Phoenix Tuner X's generated `TunerConstants.java`.

## 1. Goal

Deliver a ready-to-drive, field-centric swerve teleop template using one Xbox controller and Phoenix 6. The robot-specific hardware configuration must come exclusively from an **unmodified 2026 Phoenix Tuner X Swerve Project Generator `TunerConstants.java`**. Normal behavior settings may be configured in the template's `Constants.java`.

Keep the current manually ticked FSM and `TimedRobot` mode-loop architecture. The template should not require generated-file edits, a command-based rewrite, or robot-specific changes to `Robot.java`, the drivetrain FSM, or the controller input class after the generated constants file is supplied.

## 2. Repository context

- Java project targeting Java 17, using GradleRIO 2026.2.1 and Phoenix 6 vendordep 26.1.0.
- `Robot` manually updates FSM systems from the WPILib lifecycle callbacks; no WPILib command scheduler is used for the current driving loop.
- `DrivetrainFSMSystem` already handles field-relative chassis requests and mode-stop behavior, but the preceding partial draft's Xbox/deadband and Tuner integration must be reconciled to these requirements.
- The repository contains an old wrapper at `frc.robot.generated.CommandSwerveDrivetrain` and a newer draft at `frc.robot.subsystems.CommandSwerveDrivetrain`. The final design must have one clear template wrapper at the package/path expected by the unmodified Tuner X output; eliminate or otherwise resolve the confusing duplicate.
- Vision code exists in `VisionFSMSystem`, `LimelightHelpers`, and `Constants`, but the first test robot has no Limelight.
- The current scaffold is not robot hardware configuration. It must remain explicitly marked nonfunctional or be replaced by the generated file; never imply that the scaffold can safely control a robot.

## 3. Confirmed product decisions

| Area | Decision |
|---|---|
| Controller | One Xbox controller, Driver Station USB port 0 by default; port is configurable in `Constants.java`. |
| Drive coordinate frame | Field-centric only. No robot-centric mode or field/robot toggle. |
| Sticks | Left stick controls translation; right stick X controls yaw. Response is linear after deadband. |
| Heading reset | Xbox Start button, on press edge, manually resets the field-centric heading. Do not automatically reset heading on teleop start. |
| X-lock | Excluded. Do not add an X-lock binding or infer a button for it. |
| Speed mode | Direct full-range control; no precision mode, input ramp/slew limiter, or exponential stick curve. |
| Translation deadband | Independent per-axis deadband, default 0.08, configurable in `Constants.java`. Do not impose a circular deadband or a radial translation cap in the template input mapping. |
| Speed limits | Provide a configurable maximum linear-speed cap in `Constants.java`; its effective default is the generated `kSpeedAt12Volts` speed. Derive maximum angular speed from the configured maximum linear speed and maximum module radius. Apply translation and rotation limits independently; no additional app-level combined chassis-speed limiter is required. |
| Drive request | Phoenix `SwerveRequest.FieldCentric` with open-loop voltage for teleop. |
| Robot architecture | Keep existing FSM/mode loops; no command-based migration. |
| Autonomous | No autonomous path follower or autonomous driving routine in this scope. Command zero speed in autonomous. |
| Limelight | Keep existing vision support, but gate it behind one `Constants.java` boolean defaulting false. Off means no `VisionFSMSystem` construction and no camera/NetworkTables writes. Enabling it later should use existing camera constants/configuration. A missing camera must not prevent drivetrain operation. |
| Controller disconnect | Treat a disconnected driver controller as invalid input and command stop until it reconnects. |
| Dashboard | No new status dashboard in this scope. Retain the existing `Field2d` simulation display. |
| Init failure | Report drivetrain/Tuner initialization failure clearly. Ensure no drive requests are issued when initialization failed; simulation may warn, real robot reports an error and stays without drivetrain control. |
| Hardware verification | Include a physical safety checklist in documentation, especially for initial elevated-wheel tests, direction/offset checks, field orientation, and disable behavior. |

## 4. Xbox control mapping

Use WPILib `XboxController` and expose named control values through the existing input abstraction. Do not expose the HID object to FSM systems.

| Input | Normalized command | Axis handling |
|---|---|---|
| Left stick Y | Forward/back | Negate WPILib `getLeftY()` so pushing forward is positive. |
| Left stick X | Left/right strafe | Negate `getLeftX()` so pushing left is positive. |
| Right stick X | Counter-clockwise/clockwise rotation | Negate `getRightX()` so pushing left commands positive CCW rotation. |
| Start press | Reset field-centric forward | Edge-triggered once per press; calls Phoenix `seedFieldCentric()`. |

For each axis, clamp to `[-1, 1]`, apply the configured deadband with a rescaled linear response outside it, then multiply by that axis's speed limit. A disconnected controller must produce a zero drive command even if HID values are stale/nonzero. Other buttons are unassigned in this scope.

### Speed limits

- The generated Tuner X `kSpeedAt12Volts` is the hardware-specific measured free-speed source.
- Expose a driver maximum speed cap in `Constants.java`; the default effective value must equal that generated speed, with a straightforward option for the robot team to lower it.
- Compute the maximum module radius from generated module locations. Configure the angular speed limit as `maxLinearSpeed / maxModuleRadius` radians per second.
- Reject non-finite or non-positive values/module geometry during initialization; do not silently construct a working drive configuration from invalid generated data.
- Apply the independent axis limits described above. Do not add a new coupled translation/rotation limiter in this implementation.

## 5. Phoenix Tuner X one-file contract

### Required workflow

1. Keep all reusable drivetrain behavior in the template's normal source files.
2. Keep a clearly nonfunctional compile-time `TunerConstants.java` scaffold if needed for a clean starter checkout.
3. The robot owner replaces only `src/main/java/frc/robot/generated/TunerConstants.java` with the output from their 2026 Phoenix Tuner X generator.
4. The generated output must remain untouched: no follow-up edits to package/imports, constants, factory method, or nested generated drivetrain class.
5. The generated `createDrivetrain()` must construct the template-provided `frc.robot.subsystems.CommandSwerveDrivetrain` using Tuner's own drivetrain/module constants.

### Compatibility assumptions to validate

The official 2026 Tuner X example inspected during research emits a `TunerConstants` in `frc.robot.generated` that:

- imports `frc.robot.subsystems.CommandSwerveDrivetrain`;
- publishes `kSpeedAt12Volts`;
- provides a `createDrivetrain()` factory;
- defines nested `TunerSwerveDrivetrain` constructors that the template wrapper can extend; and
- includes the actual CAN bus, device constructors/IDs, module constants, offsets, gains, and module locations.

Therefore, keep the wrapper in `frc.robot.subsystems` and make it inherit/adapt the generated nested drivetrain type. Do not require the user to copy Tuner X's generated `CommandSwerveDrivetrain.java` into the template. The wrapper must provide any template-owned functionality (open-loop request, field perspective, vision timestamp conversion, and desktop simulation) without requiring edits to `TunerConstants.java`.

This contract is tied to the project's current 2026 dependency set and the standard 2026 Tuner X output. The user's own generator output has not yet been supplied; inspect it before claiming verified compatibility if it differs from the official output examined.

### Scaffold behavior

The placeholder `TunerConstants.java` contains no real IDs or module geometry and must never issue hardware requests. On startup its explicit unsupported-operation failure should be reported and drive FSMs should not initialize. README documentation must tell the user to replace it before deploying or expecting swerve simulation.

## 6. Alliance and field-centric behavior

- Use Phoenix operator-perspective support for field-centric controls.
- Before alliance data is available, assume the Blue Alliance forward direction so teleop remains usable.
- When a valid alliance is received, apply that alliance's operator forward direction while the robot is disabled.
- Do not change the operator perspective while enabled; in particular, do not allow delayed alliance data to abruptly flip active driver controls.
- Do not automatically seed/reset heading in `teleopInit`.
- A Start press explicitly calls Phoenix `seedFieldCentric()` and should be documented as requiring the robot to be pointed in the intended driver-forward direction when pressed.

## 7. FSM and robot-mode behavior

- Teleop: read the Xbox controls every cycle, check controller connection, apply normalized input mapping/deadband/speed limits, and issue the field-centric open-loop request.
- Disabled: command zero drivetrain demand; continue updating the alliance perspective when possible.
- Autonomous: command zero drivetrain demand. Path following and autonomous selection are explicitly out of scope.
- Test: command zero drivetrain demand unless a separately requested test feature is added later.
- Failed Tuner/drivetrain initialization: leave both drive and optional vision systems unavailable; guard callbacks with null/valid initialization checks and issue no motor requests.
- Continue using the existing explicit mode-loop pattern; do not add dependencies or migrate to WPILib command-based scheduling.

## 8. Optional Limelight behavior

Add/use one boolean in `Constants.java`, default false. When false:

- do not instantiate `VisionFSMSystem`;
- do not set a pipeline, LED mode, camera pose, or MegaTag2 robot orientation in NetworkTables; and
- do not require a connected camera for the robot to initialize or drive.

When true, preserve the existing MegaTag2 validation/fusion behavior and configure it from `Constants.java` (at least camera name and AprilTag pipeline, with existing optional transform/quality settings available). Vision remains a separate optional feature and must not be required for swerve teleop. Vision measurements may continue to be applied through Phoenix's estimator when configured.

## 9. Simulation and dashboard

- Retain Phoenix's own swerve simulation and the existing `Field2d` named `Field`.
- Do not create a second drivetrain physics/odometry model.
- Limelight detections are not synthesized in simulation; default-off vision avoids phantom camera writes/detections.
- Keep new dashboard/status displays out of scope.

## 10. Documentation requirements

Update README to plainly explain:

- only `generated/TunerConstants.java` needs robot-specific replacement;
- the exact expected generator year/version, package, wrapper import, and factory API;
- controller port and stick/Start mappings;
- field-centric/alliance behavior, including blue fallback and manual heading reset;
- deadband and maximum speed settings in `Constants.java`;
- default-off vision toggle and lack of autonomous path following;
- that the scaffold is not functional and must be replaced before deploying; and
- a safety checklist for testing on blocks before floor operation.

### Minimum physical checklist

1. Confirm the generated Tuner file matches this project's Phoenix version and uses the template wrapper import without local edits.
2. Verify CAN IDs/bus, module order/locations, motor inversions, encoder offsets, and measured free speed in Phoenix Tuner before enabling drive output.
3. First test with the robot safely raised so wheels cannot propel it. Verify neutral/disconnected-controller stop, individual wheel/steer response, and controller directions.
4. Check field-centric forward/strafe and red/blue driver perspective at low speed; confirm Start resets heading only when intentionally pressed.
5. Verify disable/autonomous stop behavior and reconnect behavior before putting the robot on the floor.

## 11. Verification and acceptance criteria

### Automated verification

- Add/adjust unit tests for pure controller-axis sign mapping, per-axis deadband/rescaling/clamping, generated speed/radius validation, and drivetrain stop/fallback behavior where testable without hardware.
- Test that a disconnected controller yields zero drive demand.
- Test the vision flag defaults false and that the disabled path does not construct or call Limelight helpers (prefer dependency seams or testable config gating over NetworkTables-dependent tests).
- Run `./gradlew test build` using the project's supported Java 17 runtime.
- Compile against an unmodified 2026 Tuner X-generated file, not only the repository scaffold, before claiming the one-file integration contract is proven.

### Functional acceptance

1. Replacing just `TunerConstants.java` with standard 2026 Tuner X output is sufficient to compile and initialize the Phoenix swerve drivetrain.
2. No IDs, module dimensions, offsets, or gains are duplicated in `Constants.java` or other robot source files.
3. With an Xbox on port 0, forward, strafe, and yaw directions match the mapping above; full stick reaches configured per-axis limits and values around neutral are suppressed by the configured deadband.
4. Start is the only field-heading reset input; no automatic teleop reset, field/robot toggle, precision mode, ramping, X-lock, or auto routine is added.
5. Unknown alliance starts with blue perspective; perspective updates only while disabled and never flips while enabled.
6. Disconnecting the controller immediately yields stop; the robot remains stopped until the controller is connected and provides input.
7. Autonomous and disabled modes issue zero drive requests.
8. Vision is fully side-effect-free while its default-off constant is false, and can be enabled later using constants without modifying drivetrain code.
9. Invalid/missing Tuner setup produces a visible startup diagnostic and no attempted drive output.
10. README contains the one-file workflow and the physical checklist.

## 12. Research references

- Phoenix 6 Swerve Builder API and 2026 Java example: <https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/mechanisms/swerve/swerve-builder-api.html>
- Phoenix 6 field-centric/operator perspective and heading seed guidance: <https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/mechanisms/swerve/using-swerve-api.html>
- Official 2026 Tuner X example `TunerConstants.java`: <https://raw.githubusercontent.com/CrossTheRoadElec/Phoenix6-Examples/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/generated/TunerConstants.java>
- Official example wrapper `CommandSwerveDrivetrain.java`: <https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithChoreo/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java>
- WPILib Xbox controller API: <https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/XboxController.html>

## 13. Known verification caveat

A prior `./gradlew test build` attempt in the current environment failed during Gradle task creation, before compilation or tests, with `Type T not present`. `./gradlew --version` showed Gradle 8.11 running on Java 26.0.2, despite the project targeting Java 17. Re-run verification using a supported Java 17 runtime before attributing the failure to project code.
