# FSMBotTemplate

A Java 17 WPILib robot template with a manually updated finite-state-machine drivetrain, Xbox teleop, PathPlanner autonomous, and optional Limelight vision. Drivetrain hardware comes from an unmodified Phoenix Tuner X-generated `TunerConstants.java` file; autonomous also requires robot-specific settings and routines from the PathPlanner GUI.

## Phoenix Tuner X integration

Replace `src/main/java/frc/robot/generated/TunerConstants.java` with this robot's **2026 Phoenix Tuner X Swerve Project Generator** output. The generated class must retain the `frc.robot.generated` package and its normal `createDrivetrain()` factory, and import the template's `frc.robot.subsystems.CommandSwerveDrivetrain`. Keep the template wrapper; do not copy Tuner X's generated wrapper file over it or edit the generated `TunerConstants.java` afterward.

That generated file supplies the hardware-specific CAN bus, device constructors and IDs, module positions, encoder offsets, motor gains, and measured `kSpeedAt12Volts`. The reusable wrapper and FSM provide open-loop-voltage field-centric control, alliance operator perspective, Phoenix pose-estimator time conversion, and Phoenix desktop simulation. No robot IDs, offsets, dimensions, or gains should be duplicated in the template `Constants.java`.

The repository's `TunerConstants.java` is a **compile-time scaffold only**: it has no hardware configuration, and its factory throws intentionally. Replace it before deploying or expecting swerve simulation. This template's integration contract targets GradleRIO 2026.2.1, the installed Phoenix 6 2026 vendordep, and standard unmodified 2026 Tuner X output. Verify compatibility against the exact output file your generator creates before your first robot test.

## Xbox controls

Connect the driver Xbox to Driver Station USB port 0. An optional mechanism Xbox uses port 1; driving does not require it. Drive mapping:

- Left stick Y: forward/backward.
- Left stick X: left/right strafe.
- Right stick X: counter-clockwise/clockwise rotation.
- Start button: reset field-centric forward on press. Point the robot toward the intended driver-forward direction before pressing Start.

Stick response is linear after an independently applied per-axis deadband. A disconnected controller commands stop; driving resumes only after reconnection and valid stick input. There is no robot-centric toggle, precision mode, slew/ramp limiter, or X-lock binding.

In `Constants.RobotIntegration`:

- `DRIVER_CONTROLLER_PORT` selects the driver port (default `0`).
- `MECHANISM_CONTROLLER_PORT` selects the mechanism port (default `1`).
- `DRIVER_DEADBAND` is the normalized stick deadband (default `0.08`).
- `MAX_LINEAR_SPEED_SCALE` scales the Phoenix Tuner X measured free speed; `1.0` uses the full generated speed and lower positive values provide a cap. Maximum angular speed is derived from the scaled linear limit divided by the generated maximum module radius.

Field-centric controls use operator perspective. Blue forward is assumed until alliance data is available. Alliance perspective updates only while disabled, so late alliance data cannot unexpectedly flip controls during an enabled period. Teleop start does not automatically reset the gyro/heading.

## Teleop-only input API

The controller owner is `frc.robot.input.TeleopInput`; FSMs consume the read-only `frc.robot.input.Input` API and the team's existing `InputTypes` enums:

```java
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.input.InputTypes.AxialInput;

public void update(Input input) {
    if (input != null && input.getButtonPressed(ButtonInput.INTAKE_BUTTON)) {
        setCurrentState(IntakeFSMState.INTAKE_STATE);
    }
    // Run state output handlers in BOTH modes, including when input is null.
}
```

`getButtonValue()` means held, `getButtonPressed()` means rising edge, and `getButtonReleased()` means falling edge. These getters do not consume edges. Robot calls `synchronize()` at teleop entry and `poll()` once at the start of each teleop loop. All consumers see the same cached values until the next poll; FSMs must never poll controllers themselves. No BooleanEvent/EventLoop objects or synthetic autonomous input are used. There is no `AutoInput` or public button/axis setter.

Buttons already held at teleop entry or reconnect are suppressed (including their held value) until released; that release does not synthesize a release edge. A subsequent press works normally. Disconnected controllers have zero axes and false buttons, without artificial press/release edges. Axes resume at their current value on reconnect; there is no neutral-stick rearming. Nonfinite axes become zero and other values are clamped to [-1, 1]. Input does not apply drivetrain deadband a second time.

**Mechanism disconnect behavior:** loss of the mechanism controller resets all registered mechanisms immediately and skips their teleop updates while disconnected. The driver remains independent. Reconnection resumes mechanism updates with held buttons suppressed until release. Safe output still depends on each FSM's reset implementation. Autonomous mechanism updates do not depend on either controller being connected.

Bindings below expose logical actions only. Apart from drivetrain sticks and reseed, the template does not implement the corresponding robot features—you supply their FSM behavior.

| Controller | Physical control | Logical input |
| --- | --- | --- |
| Driver | Left Y (negated) | `DRIVETRAIN_DRIVE_X` — positive forward |
| Driver | Left X (negated) | `DRIVETRAIN_DRIVE_Y` — positive left |
| Driver | Right X (negated) | `DRIVETRAIN_ROTATE` — positive counterclockwise |
| Driver | Start / Y / Back | `DRIVETRAIN_RESEED` / `DRIVETRAIN_PATHFIND` / `DRIVETRAIN_INVERT_CONTROLS` |
| Driver | Left / right trigger | `FACE_PASS` / `FACE_HUB` |
| Driver | X / A | `BALL_SHAKE_FRONT` / `BALL_SHAKE_SIDE` |
| Mechanism | Left X | `CLIMBER_MANUAL_CONTROL` — positive right |
| Mechanism | B / A | `INTAKE_BUTTON` / `OUTTAKE_BUTTON` |
| Mechanism | Y / X / Start | `FOLD_IN_BUTTON` / `FOLD_OUT_BUTTON` / `PARTIAL_OUT_BUTTON` |
| Mechanism | POV up / left / down | `CLIMBER_MANUAL_OVERRIDE` / `CLIMBER_NEXT_STEP` / `CLIMBER_EMERGENCY_ABORT` |
| Mechanism | Right / left bumper | `PASSER_PREP_TOGGLE` / `MANUAL_SHOOT_TOGGLE` |
| Mechanism | Left / right trigger | `SHOOTER_PREP_TOGGLE` / `REV_FEEDER` |
| Mechanism | Back | `IDLE_SHOOTER_TOGGLE` |
| Unbound | None | `CLIMBER_AUTO_UP_1`, `CLIMBER_AUTO_UP_2` |

Trigger actions become true above 0.5. The drivetrain coordinate signs intentionally preserve this template's behavior rather than the raw signs of last season's input adapter.

**Migration:** mechanisms can keep `update(Input)`, `nextState(Input)`, `getCurrentState()`, public `setCurrentState()`, and their own enum names. Replace root-package `frc.robot.TeleopInput` imports with `frc.robot.input.Input` in FSMs. The controller-owning class now lives in `frc.robot.input`. Do not carry over `if (input == null) return;` from the old intake: it prevents autonomous outputs and reset outputs. For button-only transitions, null input should preserve the current state; sensor/timer transitions should remain available in either mode. `watchForStatesCommand()` from the old season base is not included in this input change.

## Robot modes

Teleop uses the Xbox FSM mapping above. Autonomous uses PathPlannerLib 2026.1.2 and WPILib's command scheduler, ticked only in `autonomousPeriodic()`. No teleop default command is registered, and the drivetrain teleop FSM does not run during an active auto. Registered mechanism FSMs share their normal state handlers across teleop and autonomous. Disabled and test modes command zero drive demand. Auto completion, cancellation, mode changes, and a 15-second deadline cancel autonomous/event commands and stop the drivetrain. A runtime scheduler failure stops autonomous and prevents retries until robot code is restarted; teleop remains independent.

## PathPlanner autonomous setup

1. Open this robot project in the **2026 PathPlanner GUI** and save its robot configuration. It must produce `src/main/deploy/pathplanner/settings.json`. Configure robot mass, moment of inertia, wheel radius, drive gearing, measured maximum drive speed, motor type, friction coefficient, and current limit for the actual robot. Do not use fabricated physics values or reduce the physical maximum speed just to slow an auto.
2. Select holonomic mode and enter module locations in **front-left, front-right, back-left, back-right** order, matching TunerConstants. Startup rejects nonpositive/nonfinite physical limits or module geometry that differs by more than 1 mm. Other hardware/physics settings still require manual verification.
3. Create paths and an auto in the GUI; save them under `src/main/deploy/pathplanner/paths/*.path` and `src/main/deploy/pathplanner/autos/*.auto`. Enable **Reset Odometry** on the auto so PathPlanner seeds the starting pose. Physically place the robot at that pose and heading before enabling. Keep the final path's end velocity at zero for the first test.
4. Use conservative path velocity/acceleration limits for the first floor test (for example, 0.5 m/s and 0.5 m/s²). **`MAX_LINEAR_SPEED_SCALE` limits teleop only, not PathPlanner autonomous.** Tune `Constants.Autonomous.TRANSLATION_KP` and `ROTATION_KP` on the actual robot; the initial values are not guaranteed to suit every drivetrain. Phoenix autonomous uses closed-loop velocity control, so generated drive gains must be tuned too.
5. Build and deploy only after completing the hardware checks below. Select a saved routine using dashboard **Auto Chooser**, verify **Auto/Ready**, and enable autonomous from Driver Station. The default **None** selection does not move. Missing/invalid PathPlanner configuration leaves autonomous unavailable without blocking teleop. No moving auto or guessed robot settings are included in the template.

PathPlanner reads Phoenix's estimated blue-origin pose and measured robot-relative speeds, resets the estimator pose when the auto requests it, and sends robot-relative velocities plus X/Y wheel-force feedforwards to Phoenix. Red-alliance routines are automatically flipped; unknown alliance assumes blue, so confirm Driver Station alliance before enabling. Operator perspective affects teleop only and is not used for autonomous output. Vision is optional for following paths.

The global command scheduler is reserved for autonomous in this template. Register PathPlanner named/event commands **before** creating the auto chooser. The old `AutoHandlerSystem`, `AutoFSMState`, and `updateAutonomous()` API have been removed: autonomous commands directly select the shared FSM's states.

**Deployment note:** `build.gradle` now sets `deleteOldFiles = true` for static deploy files, following PathPlanner's recommendation so removed autos cannot remain in the chooser. A future deploy removes roboRIO deploy-directory files absent from `src/main/deploy`; keep any required static files in the project. No deploy is run by a build.

References: [PathPlanner autonomous integration](https://pathplanner.dev/pplib-build-an-auto.html), [robot configuration](https://pathplanner.dev/robot-config.html).

## Shared mechanism FSMs in autonomous

`FSMSystem<S>` exposes `setState(S state)` and `setStateCommand(S state)`. Both directly assign the mechanism's actual state, without transition guards, goal mapping, or state locking. Null states are rejected. Normal sensor/timer-driven transitions remain active after assignment. Commands change state at initialization, not when constructed, and never tick the FSM themselves.

For robot-specific mechanisms, populate `Robot.initializeMechanisms()`:

```java
// Inside Robot; IntakeFSMSystem represents YOUR robot's implementation.
@Override
protected void initializeMechanisms() {
    IntakeFSMSystem intake = new IntakeFSMSystem();
    registerMechanism(intake);
    NamedCommands.registerCommand("Intake/Run",
        intake.setStateCommand(IntakeFSMSystem.State.INTAKE));
    NamedCommands.registerCommand("Intake/Stop",
        intake.setStateCommand(IntakeFSMSystem.State.STOPPED));
}
```

Import `com.pathplanner.lib.auto.NamedCommands` and your mechanism class. Edit this hook in `Robot` (omit `@Override` when editing the existing method), or override it in your own Robot subclass if that subclass is the configured robot entry point. The hook runs before the auto chooser loads paths/routines, even when the drivetrain scaffold is still present. The template's hook is empty: **it does not construct example motor hardware or invent an intake**. Duplicate mechanism instances, late registrations, and drivetrain/vision registrations are rejected.

You can also directly call:

```java
intake.setState(IntakeFSMSystem.State.INTAKE);
```

Loop ownership:

- **Teleop:** Robot polls its teleop snapshot first, then calls each registered mechanism's `update(Input input)` once per loop while the mechanism controller is connected.
- **Autonomous:** PathPlanner processes commands/events first, then Robot calls each mechanism's `update(null)` once while the routine remains active. Null means no controller input, not an instruction to stop. State handlers must work without a controller; only controller-driven transitions require non-null input.
- **Disabled/test:** moving mechanism handlers are not ticked. Lifecycle resets immediately apply their safe outputs.
- Mechanisms implement WPILib `Subsystem` only to supply shared command requirements. Do not add a scheduler `periodic()` or default command that also ticks them; Robot already owns updates. The drivetrain and vision retain their dedicated lifecycle and must not be added to the mechanism list.

In the PathPlanner GUI, use named commands **Intake/Run** and **Intake/Stop** as auto steps or path event markers. For example:

```text
Intake/Run → Follow pickup path → Intake/Stop
```

A path marker can change the intake state while the robot drives. State commands require their mechanism, not the drivetrain. PathPlanner also includes marker mechanism requirements in the enclosing path command, so avoid competing parallel actions for that same mechanism during the path.

**One-shot semantics:** setting state finishes immediately; it does not wait for physical completion or hold exclusive ownership after finishing. The state persists until another setter, an internal transition, or lifecycle reset. Back-to-back setters can replace an intermediate state before a handler executes. A routine consisting only of a state setter can finish and reset before any mechanism output update. Use a GUI wait, a path, or a later marker if the action needs time. A timed wait is not sensor-confirmed completion.

**Reset contract:** each mechanism's `reset()` must immediately set its safe state **and apply safe motor/actuator outputs**, not just change the enum. `ExampleFSMSystem.reset()` demonstrates applying the start-state output with `update(null)`. Resets happen before auto scheduling and after completion, cancellation, the 15-second deadline, runtime failure, disable, teleop/test entry, and shutdown. They must be safe to call repeatedly. Resetting all registered mechanisms is attempted even when one fails. A mechanism update/reset failure cancels autonomous and latches shared mechanism updates off until robot code restarts; drivetrain teleop remains independent. Cancellation of an already finished one-shot command alone cannot stop a motor—the lifecycle reset does that.

Migration: remove any old `updateAutonomous(AutoFSMState)` override/import from robot-specific mechanisms, expose their state enum publicly, register their state commands in the initialization hook, and keep the actual action logic in their existing `update()`/state handlers. The example exposes `ExampleFSMSystem.FSMState`, but constructing it still creates its example Spark hardware and requires verifying that configuration first.

## Optional Limelight vision

Vision support remains available in `VisionFSMSystem`, but is disabled by default. Set `Constants.RobotIntegration.ENABLE_LIMELIGHT_VISION` to `true` only when a Limelight is configured. When false, the robot does not construct the vision FSM or write camera pipeline, LED, camera pose, or MegaTag2 orientation values to NetworkTables; a camera is not required to initialize or drive.

When enabled, default camera name and AprilTag pipeline are in `Constants.RobotIntegration`. `VisionFSMSystem.Config` also supports an optional camera-to-robot transform, field bounds, quality thresholds, pose uncertainty, and coordinate transform. The default camera transform assumes it has already been configured in Limelight Web UI. Valid MegaTag2 measurements are checked and fused into Phoenix's estimator. MegaTag2 yaw uses the estimator's field-aligned heading, including PathPlanner starting-pose resets, rather than unadjusted Pigeon yaw. Runtime vision exceptions disable vision for the boot cycle without preventing drive updates. `imported/LimelightHelpers` is a focused NetworkTables adapter, not the full upstream helper library. Limelight detections are not synthesized in simulation.

## Simulation and dashboard

The existing GradleRIO Sim GUI/Driver Station and Phoenix swerve simulation are used together; the template does not create a second physics model. WPILib `Field2d` named `Field` displays the Phoenix estimated pose on hardware and in simulation. The dashboard also exposes `Auto Chooser` and `Auto/Ready`.

## Before driving on carpet

1. Confirm the generated Tuner file is the 2026 output for this project's Phoenix version, uses the template wrapper import, and has not been locally edited.
2. In Phoenix Tuner, verify the CAN bus and IDs, module order and positions, motor inversions, encoder offsets, and measured free speed. Check that generated module geometry is finite and nonzero.
3. Securely raise the robot so no wheel can propel it. Verify safe neutral behavior, the disconnected-controller stop and reconnection, steering, each module's direction, and the Xbox forward/strafe/rotation signs.
4. At low speed, verify blue and red field-centric forward/strafe behavior. Confirm Start resets field heading only when intentionally pressed and while the robot is pointed at the intended driver-forward direction.
5. Verify disabled/test modes and autonomous with **None** selected stop the drivetrain. Resolve initialization diagnostics before attempting floor operation.
6. After elevated checks pass, perform the first teleop floor test at reduced `MAX_LINEAR_SPEED_SCALE` with a clear area and a person ready to disable the robot.
7. Test a short, low-speed PathPlanner auto in a clear area from its configured starting pose. Verify blue/red flipping, completion stop, immediate disable, and teleop takeover before using longer routines.

## Build and simulation commands

- `./gradlew simulateJava` — desktop simulation (requires a valid generated Tuner file for simulated drivetrain operation).
- `./gradlew test build` — test and build (Java 17).
- On this Mac: `./gradlew build -Dorg.gradle.java.home="/Users/arnavshah/wpilib/2026/jdk"` — build and run the tests using the WPILib JDK.
- Deploy only after replacing the placeholder `TunerConstants.java` with the correct robot-specific generated file and completing the physical checks above.
