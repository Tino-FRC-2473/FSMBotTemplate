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

	/**
	 * Read a normalized axis value from the most recent sample.
	 * @param key axis to read
	 * @return value in [-1, 1], or zero when disconnected or the raw value was nonfinite
	 */
	public final double getAxisValue(AxialInput key) {
		return axes[Objects.requireNonNull(key, "axis").ordinal()];
	}

	/**
	 * Whether the button is currently held.
	 * @param key button to read
	 * @return true while the button is held
	 */
	public final boolean getButtonValue(ButtonInput key) {
		return held[Objects.requireNonNull(key, "button").ordinal()];
	}

	/**
	 * Whether the button went down since the previous sample. Does not consume the edge.
	 * @param key button to read
	 * @return true only during the sample after a rising edge
	 */
	public final boolean getButtonPressed(ButtonInput key) {
		return pressed[Objects.requireNonNull(key, "button").ordinal()];
	}

	/**
	 * Whether the button went up since the previous sample. Does not consume the edge.
	 * @param key button to read
	 * @return true only during the sample after a falling edge
	 */
	public final boolean getButtonReleased(ButtonInput key) {
		return released[Objects.requireNonNull(key, "button").ordinal()];
	}

	/**
	 * Whether the driver controller is currently connected.
	 * @return true while the driver controller is connected
	 */
	public final boolean isDriverControllerConnected() {
		return driverConnected;
	}

	/**
	 * Whether the mechanism controller is currently connected.
	 * @return true while the mechanism controller is connected
	 */
	public final boolean isMechanismControllerConnected() {
		return mechanismConnected;
	}

	// Package-private mutation is reserved for physical teleop sampling, not subsystem commands.
	final void beginSample(boolean driver, boolean mechanism, boolean synchronize) {
		for (ButtonInput button : ButtonInput.values()) {
			int index = button.ordinal();
			boolean connected = switch (button.controller()) {
				case DRIVE -> driver;
				case MECH -> mechanism;
			};
			boolean previouslyConnected = switch (button.controller()) {
				case DRIVE -> driverConnected;
				case MECH -> mechanismConnected;
			};
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
		boolean connected = switch (button.controller()) {
			case DRIVE -> driverConnected;
			case MECH -> mechanismConnected;
		};
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
		boolean connected = switch (axis.controller()) {
			case DRIVE -> driverConnected;
			case MECH -> mechanismConnected;
		};
		axes[axis.ordinal()] = connected && Double.isFinite(raw)
			? Math.max(-1.0, Math.min(1.0, raw)) : 0.0;
	}
}
