# Example FSM

The example mechanism lives in `doc/examples/ExampleFSMSystem.java` and is **quarantined**: it is
outside `src/main/java`, so it is not compiled, not deployed, and not checked by checkstyle.

It was moved here because the template's `Robot.initializeMechanisms()` is intentionally empty, so
nothing ever constructed it — but its constructor opened a Spark on CAN 5, and a stale example is
worse than no example.

## What it demonstrates

- One mechanism, one `FSMSystem` subclass, one nested state enum.
- `reset()` selecting the safe start state and applying that state's output with a single
  `update(null)` tick, rather than only changing the enum.
- A `nextState()` that reads only `input` and the current state.
- Handlers that write output for their own state only, with a `default` that throws.

## Using it

1. Copy the file into `src/main/java/frc/robot/systems/` and rename the class and state enum for the
   real mechanism (`IntakeFSMSystem` / `IntakeFSMState`).
2. Replace the motor construction with the real wrapper and CAN ID, and move the ID into
   `HardwareMap`.
3. Move `MOTOR_RUN_POWER` and any setpoints into `Constants.<Name>Constants`.
4. Register it in `Robot.initializeMechanisms()`, and register any `setStateCommand(...)` with
   `NamedCommands` before the auto chooser is built.

Because the file is not compiled, its imports are not verified against the current WPILib and vendor
APIs. Expect to fix them when you copy it back.
