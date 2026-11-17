# Vision FSM

Owner: `frc.robot.systems.VisionFSMSystem`, created by `Robot` only when
`Constants.RobotIntegration.ENABLE_LIMELIGHT_VISION` is true.
Source of measurements: a Limelight running MegaTag2 (`botpose_orb_wpiblue`).

Vision is **not** a shared mechanism. `Robot.updateVisionSafely()` calls `visionFSM.update(null)`
once per enabled loop (autonomous, teleop, disabled, test), and `registerMechanism` rejects the
type. `update(Input)` is the only entry point; the input argument is unused because localization
does not depend on the driver.

## States

The state is a **diagnostic output**, not a goal. `nextState()` derives it from the last
`RejectionReason`; nothing transitions into a state to make something happen.

| State | Meaning |
| --- | --- |
| `WAITING_FOR_TAGS` | No frame, and the pipeline reports no target |
| `TRACKING` | The last frame passed validation and was fused |
| `REJECTING_ESTIMATE` | A tag is visible or a frame arrived, but nothing usable came out of it |

| `RejectionReason` | Resulting state |
| --- | --- |
| `NONE` | `TRACKING` |
| `NO_TAGS` | `WAITING_FOR_TAGS` |
| every other reason | `REJECTING_ESTIMATE` |

## Tick order

1. Publish orientation with `SetRobotOrientation_NoFlush`: estimator heading, measured yaw rate,
   and Pigeon roll/pitch. MegaTag2 needs this before the frame is read.
2. Read `botpose_orb_wpiblue` (MegaTag2) and `botpose_wpiblue` (tag-only) for the same frame.
3. `validateIncoming`: null/no tags, invalid timestamp, duplicate timestamp (in that order), then
   **consume the timestamp** and run `validateEstimate`.
4. `fuseIfValid`: apply `poseTransform`, reject a null or nonfinite result, then either seed the
   drivetrain pose (first frame that also passes the seed-quality gate) or hand the clamped pose and
   standard deviations to the drivetrain's `addVisionMeasurement`.
5. `setCurrentState(nextState(...))`, then log once.

## Timestamp contract

A frame is fused at most once.

- Timestamps must be finite, positive, and strictly greater than the last processed value.
- A frame that fails validation **still consumes** its timestamp, so a bad frame is never retried
  on a later loop. This is deliberate: retrying would make a stale estimate look fresh.
- `reset()` preserves the last timestamp, so a mode change cannot re-fuse a frame that was already
  applied.
- The fused timestamp is clamped to `now - FUSE_MARGIN_SECONDS` at the latest, because the estimator
  silently drops a measurement newer than its newest odometry sample.

## Rejection reasons

| Reason | Trigger |
| --- | --- |
| `NONE` | Accepted |
| `NO_TAGS` | Estimate is null or `tagCount <= 0`, and the pipeline reports no target (`tv` false) |
| `DETECTED_NO_POSE` | Pipeline has a target (`tv` true) but MegaTag2 returned no pose — check the uploaded field map, the robot-space pose, and robot orientation |
| `INVALID_FIDUCIALS` | `rawFiducials` missing, length mismatch, or a null/nonfinite entry |
| `INVALID_POSE` | Pose null or nonfinite before transform, or after `poseTransform` |
| `INVALID_TIMESTAMP` | Timestamp nonfinite or `<= 0` |
| `STALE` | Older than `maxMeasurementAgeSeconds` |
| `FUTURE_TIMESTAMP` | Further ahead than `maxFutureTimestampSeconds` |
| `DUPLICATE_TIMESTAMP` | Not strictly newer than the last processed frame |
| `OUT_OF_FIELD` | Outside `fieldBounds` when bounds are configured |
| `TOO_FAR` | `avgTagDist` nonfinite, negative, or beyond `maxAverageTagDistanceMeters` |
| `HIGH_AMBIGUITY` | No tag at or below `maxTagAmbiguity` |

## Seeding

Filtering alone cannot localize the robot. The estimator keeps an odometry-only pose that starts at
the boot position and applies vision as compensation on top of it, so any discarded compensation —
a pose or rotation reset, or a measurement the estimator itself drops — reveals `0,0` again. The
first frame that passes validation **and** the seed-quality gate therefore calls
`CommandSwerveDrivetrain.seedPoseFromVision`, which calls `resetPose`. The odometry track itself
moves into field coordinates, so losing the tag, a mode change, or a Start press can no longer
return the pose to `0,0`.

Seed quality is `tagCount >= MIN_SEED_TAGS`, or `avgTagDist <= MAX_SEED_TAG_DISTANCE_METERS` for a
single tag. `localized` records that a seed happened; `reset()` preserves it, and the seed count, the
same way it preserves the last timestamp. Every later frame fuses through `addVisionMeasurement` as
before.

## Heading

MegaTag2 solves its position using the heading published in step 1, so the rotation it reports is the
robot's own estimate coming back: fusing that theta cannot move yaw. The tag-only `botpose_wpiblue`
pose is solved from tag geometry and is independent, so its rotation is mixed with the MegaTag2
translation to form one measurement per frame.

That substitution happens only while the tag-only fix is trustworthy — the same `isSeedQualitySufficient`
gate used for seeding, plus passing `validateEstimate` (freshness, ambiguity, distance, field bounds).
When it fails, the frame keeps the gyro heading and the huge theta standard deviation. When it holds,
theta fuses at `HEADING_STD_DEV_RADIANS`, so heading is corrected over several frames instead of
teleporting, and the first such frame also seeds the pose with the tag-derived heading.

Tag-only heading depends on the camera's robot-space pose (mounting transform) being correct: set it
in the Limelight web UI or enable `configureCameraPose` with the measured transform. A wrong transform
appears as a constant yaw bias that fusion cannot remove.

## Configuration

`Config` validates in its compact constructor; `Config.defaults(...)` fills the template values.

| Field | Default | Meaning |
| --- | --- | --- |
| `limelightName` | `Constants.RobotIntegration.LIMELIGHT_NAME` | NetworkTables table name |
| `aprilTagPipeline` | `Constants.RobotIntegration.APRILTAG_PIPELINE` | pipeline index |
| `configureCameraPose` | `false` | push `robotToCamera` to the camera on startup |
| `robotToCamera` | identity | mounting transform; only used when above is true |
| `maxMeasurementAgeSeconds` | `0.50` | oldest usable frame |
| `maxFutureTimestampSeconds` | `0.05` | clock-skew tolerance |
| `maxTagAmbiguity` | `0.20` | highest per-tag ambiguity |
| `maxAverageTagDistanceMeters` | `6.0` | farthest usable average tag distance |
| `fieldBounds` | `null` | optional inclusive field rectangle |
| `measurementStdDevs` | `(0.70, 0.70, 1e6)` | x/y trusted, heading not |
| `poseTransform` | identity | optional coordinate change |

MegaTag2 supplies its own heading, so theta standard deviation is effectively infinite unless the
tag-only solve is trustworthy, in which case `HEADING_STD_DEV_RADIANS` replaces it.

## Telemetry

- `Vision/State`, `Vision/Accepted`, `Vision/RejectionReason` — one record per loop.
- `Vision/Localized`, `Vision/SeedCount` — whether the pose was ever seeded from vision, and how
  many seeds happened. `Vision/FusedTimestampAge_s` is the age of the last processed frame.
- `Vision/HeadingFromTags` — true when this frame's yaw came from the tag-only solve instead of the
  gyro.
- `Vision/EstimatedPose` is logged only for frames that actually carry a tag, so it never shows a
  fabricated origin.
- `Vision/TagCount`, `Vision/TimestampSeconds`, `Vision/EstimatedPose` — from the frame, when present.
- `Vision/Camera/HasTarget`, `Vision/Camera/TargetID`, `Vision/Camera/Heartbeat`,
  `Vision/Camera/Pipeline` — raw camera state, read every loop. `HasTarget` is the pipeline's
  `tv`; it is true whenever a tag is detected, independently of MegaTag2 localization output.
  A flat `Heartbeat` means robot and camera are not talking (wrong `limelightName`, network).
  It also shows the pipeline the camera reports as active, which is not necessarily the one the
  robot requested.

## Failure handling

`Robot.updateVisionSafely()` wraps `update(null)` in a try/catch. Vision is never required to drive
or to follow a path.

A read that throws is retried on the next loop rather than ending vision. NT4 can throw while
decoding a value the camera just published (for example `java.nio.BufferUnderflowException`); that
is transient and does not mean the camera is gone. The Driver Station reports the exception type and
the throwing frame the first time a *new* failure appears. Only
`Constants.VisionDefaults.MAX_CONSECUTIVE_READ_FAILURES` failures in a row (about 0.5 s at 50 Hz)
disable vision for the rest of the boot cycle.

If a `BufferUnderflowException` persists, the camera side is publishing something the client cannot
decode: confirm the Limelight runs LLOS 2026.0 or later (LimelightHelpers v1.14 requires it) and that
only one NT4 client/server pair is talking on the network.

## Pit checks

1. With vision disabled, confirm no NetworkTables writes appear and the robot drives normally.
2. Enable vision only with the camera configured; confirm `Vision/State` reaches `TRACKING` while a
   tag is in view and `Vision/Accepted` is true.
3. Boot with every tag covered and confirm `Vision/Localized` is false and the pose stays at the
   start pose. Uncover a tag and confirm `Vision/SeedCount` becomes 1 and the field pose jumps to the
   tag-implied location once; cover it again and drive, and confirm the pose keeps moving instead of
   returning to `0,0`.
4. With the robot on blocks, show two tags (or one within `MAX_SEED_TAG_DISTANCE_METERS`) and confirm
   `Vision/HeadingFromTags` turns true and the pose heading rotates to the tag-implied direction
   without the robot moving.
5. Confirm `Vision/RejectionReason` reads `STALE` or `DUPLICATE_TIMESTAMP` when the camera clock or
   network is misbehaving, rather than a burst of accepted frames.
6. Wave a tag in front of the camera and confirm the reported pose moves in the expected direction
   for the alliance; MegaTag2 reports the blue-origin pose.
7. Confirm the mounting transform matches the real camera if `configureCameraPose` is enabled,
   otherwise confirm it was set in the Limelight web UI.
