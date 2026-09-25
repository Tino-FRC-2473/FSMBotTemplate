package frc.robot.imported;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;

/**
 * Small Limelight NetworkTables helper surface used by the robot's AprilTag localization FSM.
 * The full LimelightHelpers distribution is available from Limelight; this class keeps only the
 * pose, orientation, pipeline, and LED operations needed by this project.
 */
public final class LimelightHelpers {
	private static final Map<String, DoubleArrayEntry> DOUBLE_ARRAY_ENTRIES = new ConcurrentHashMap<>();

	private LimelightHelpers() { }

	/** Details for one fiducial included in a Limelight pose estimate. */
	public static final class RawFiducial {
		public final int id;
		public final double txnc;
		public final double tync;
		public final double ta;
		public final double distToCamera;
		public final double distToRobot;
		public final double ambiguity;

		public RawFiducial(
			int id, double txnc, double tync, double ta,
			double distToCamera, double distToRobot, double ambiguity) {
			this.id = id;
			this.txnc = txnc;
			this.tync = tync;
			this.ta = ta;
			this.distToCamera = distToCamera;
			this.distToRobot = distToRobot;
			this.ambiguity = ambiguity;
		}
	}

	/** Timestamped robot pose estimate returned by a blue-alliance Limelight pose topic. */
	public static final class PoseEstimate {
		public final Pose2d pose;
		public final double timestampSeconds;
		public final double latency;
		public final int tagCount;
		public final double tagSpan;
		public final double avgTagDist;
		public final double avgTagArea;
		public final RawFiducial[] rawFiducials;
		public final boolean isMegaTag2;

		public PoseEstimate() {
			this(new Pose2d(), 0.0, 0.0, 0, 0.0, 0.0, 0.0, new RawFiducial[0], false);
		}

		public PoseEstimate(
			Pose2d pose, double timestampSeconds, double latency, int tagCount,
			double tagSpan, double avgTagDist, double avgTagArea,
			RawFiducial[] rawFiducials, boolean isMegaTag2) {
			this.pose = pose;
			this.timestampSeconds = timestampSeconds;
			this.latency = latency;
			this.tagCount = tagCount;
			this.tagSpan = tagSpan;
			this.avgTagDist = avgTagDist;
			this.avgTagArea = avgTagArea;
			this.rawFiducials = rawFiducials;
			this.isMegaTag2 = isMegaTag2;
		}
	}

	private static String sanitizeName(String name) {
		return name == null || name.isBlank() ? "limelight" : name;
	}

	private static NetworkTable getTable(String name) {
		return NetworkTableInstance.getDefault().getTable(sanitizeName(name));
	}

	private static NetworkTableEntry getEntry(String name, String key) {
		return getTable(name).getEntry(key);
	}

	private static DoubleArrayEntry getDoubleArrayEntry(String name, String key) {
		String cacheKey = sanitizeName(name) + "/" + key;
		return DOUBLE_ARRAY_ENTRIES.computeIfAbsent(cacheKey,
			unused -> getTable(name).getDoubleArrayTopic(key).getEntry(new double[0]));
	}

	/** Select a configured pipeline index on the Limelight. */
	public static void setPipelineIndex(String limelightName, int pipelineIndex) {
		getEntry(limelightName, "pipeline").setDouble(pipelineIndex);
	}

	/** Configure the Limelight LED mode to be controlled by the selected pipeline. */
	public static void setLEDMode_PipelineControl(String limelightName) {
		getEntry(limelightName, "ledMode").setDouble(0.0);
	}

	/** Configure the camera transform relative to the robot, in meters and degrees. */
	public static void setCameraPose_RobotSpace(
		String limelightName, double forward, double side, double up,
		double roll, double pitch, double yaw) {
		getEntry(limelightName, "camerapose_robotspace_set").setDoubleArray(
			new double[] {forward, side, up, roll, pitch, yaw});
	}

	/** Publish orientation for MegaTag2 without forcing a NetworkTables flush each robot loop. */
	public static void SetRobotOrientation_NoFlush(
		String limelightName, double yaw, double yawRate,
		double pitch, double pitchRate, double roll, double rollRate) {
		getEntry(limelightName, "robot_orientation_set").setDoubleArray(
			new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
	}

	/** Published Limelight heartbeat, used to ignore default/stale NetworkTables entries. */
	public static double getHeartbeat(String limelightName) {
		return getEntry(limelightName, "hb").getDouble(0.0);
	}

	/** Return the latest blue-alliance MegaTag2 pose estimate, or an empty estimate if unavailable. */
	public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
		TimestampedDoubleArray timestamped = getDoubleArrayEntry(limelightName, "botpose_orb_wpiblue").getAtomic();
		double[] data = timestamped.value;
		if (data == null || data.length < 11) {
			return new PoseEstimate();
		}

		double latencyMs = data[6];
		int tagCount = (int) data[7];
		if (tagCount <= 0 || data.length != 11 + (7 * tagCount)) {
			return new PoseEstimate();
		}

		RawFiducial[] fiducials = new RawFiducial[tagCount];
		for (int index = 0; index < tagCount; index++) {
			int offset = 11 + (index * 7);
			fiducials[index] = new RawFiducial(
				(int) data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
				data[offset + 4], data[offset + 5], data[offset + 6]);
		}

		// NetworkTables timestamps are in microseconds; Limelight latency is in milliseconds.
		double timestampSeconds = (timestamped.timestamp / 1_000_000.0) - (latencyMs / 1_000.0);
		Pose2d pose = new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
		return new PoseEstimate(
			pose, timestampSeconds, latencyMs, tagCount, data[8], data[9], data[10], fiducials, true);
	}
}
