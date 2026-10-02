package frc.robot.input;

import java.util.Objects;

import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;

/**
 * Read-only view of one teleop iteration, shared by all FSMs on the robot main thread.
 * Getters never poll hardware or consume edges. Only the package's controller owner updates it.
 * Autonomous uses null input and direct FSM commands, not an Input implementation.
 */
public abstract class Input {
	private final double[] axes = new double[AxialInput.values().length];
	private final boolean[] held = new boolean[ButtonInput.values().length];
	private final boolean[] pressed = new boolean[held.length];
	private final boolean[] released = new boolean[held.length];
	private final boolean[] blockedUntilRelease = new boolean[held.length];
	private boolean driverConnected;
	private boolean mechanismConnected;

	public final double getAxisValue(AxialInput key) {
		return axes[Objects.requireNonNull(key, "axis").ordinal()];
	}

	public final boolean getButtonValue(ButtonInput key) {
		return held[Objects.requireNonNull(key, "button").ordinal()];
	}

	public final boolean getButtonPressed(ButtonInput key) {
		return pressed[Objects.requireNonNull(key, "button").ordinal()];
	}

	public final boolean getButtonReleased(ButtonInput key) {
		return released[Objects.requireNonNull(key, "button").ordinal()];
	}

	public final boolean isDriverControllerConnected() {
		return driverConnected;
	}

	public final boolean isMechanismControllerConnected() {
		return mechanismConnected;
	}

	// Package-private mutation is reserved for physical teleop sampling, not subsystem commands.
	final void beginSample(boolean driver, boolean mechanism, boolean synchronize) {
		for (ButtonInput button : ButtonInput.values()) {
			int index = button.ordinal();
			boolean connected = button.usesDriverController() ? driver : mechanism;
			boolean previouslyConnected = button.usesDriverController()
				? driverConnected : mechanismConnected;
			pressed[index] = false;
			released[index] = false;
			if (!connected || synchronize || !previouslyConnected) {
				held[index] = false;
				blockedUntilRelease[index] = connected;
			}
		}
		driverConnected = driver;
		mechanismConnected = mechanism;
	}

	final void sampleButton(ButtonInput button, boolean raw) {
		int index = button.ordinal();
		boolean connected = button.usesDriverController() ? driverConnected : mechanismConnected;
		if (!connected) {
			return;
		}
		if (blockedUntilRelease[index]) {
			if (!raw) {
				blockedUntilRelease[index] = false;
			}
			return;
		}
		pressed[index] = raw && !held[index];
		released[index] = !raw && held[index];
		held[index] = raw;
	}

	final void sampleAxis(AxialInput axis, double raw) {
		boolean connected = axis == AxialInput.CLIMBER_MANUAL_CONTROL
			? mechanismConnected : driverConnected;
		axes[axis.ordinal()] = connected && Double.isFinite(raw)
			? Math.max(-1.0, Math.min(1.0, raw)) : 0.0;
	}
}
