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
2. Read one `botpose_orb_wpiblue` estimate.
3. `validateIncoming`: null/no tags, invalid timestamp, duplicate timestamp (in that order), then
   **consume the timestamp** and run `validateEstimate`.
4. `fuseIfValid`: apply `poseTransform`, reject a null or nonfinite result, then hand the pose and
   standard deviations to the drivetrain's `addVisionMeasurement`.
5. `setCurrentState(nextState(...))`, then log once.

## Timestamp contract

A frame is fused at most once.

- Timestamps must be finite, positive, and strictly greater than the last processed value.
- A frame that fails validation **still consumes** its timestamp, so a bad frame is never retried
  on a later loop. This is deliberate: retrying would make a stale estimate look fresh.
- `reset()` preserves the last timestamp, so a mode change cannot re-fuse a frame that was already
  applied.

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

MegaTag2 supplies its own heading, so theta standard deviation is effectively infinite: vision
never argues with the gyro.

## Telemetry

- `Vision/State`, `Vision/Accepted`, `Vision/RejectionReason` — one record per loop.
- `Vision/TagCount`, `Vision/TimestampSeconds`, `Vision/EstimatedPose` — from the frame, when present.
- `Vision/Camera/HasTarget`, `Vision/Camera/TargetID`, `Vision/Camera/Heartbeat`,
  `Vision/Camera/Pipeline` — raw camera state, read every loop. `HasTarget` is the pipeline's
  `tv`; it is true whenever a tag is detected, independently of MegaTag2 localization output.
  A flat `Heartbeat` means robot and camera are not talking (wrong `limelightName`, network).
  It also shows the pipeline the camera reports as active, which is not necessarily the one the
  robot requested.

## Failure handling

`Robot.updateVisionSafely()` wraps `update(null)` in a try/catch. A runtime exception clears the
reference for the rest of the boot cycle: vision stops, driving continues. Vision is never required
to drive or to follow a path.

## Pit checks

1. With vision disabled, confirm no NetworkTables writes appear and the robot drives normally.
2. Enable vision only with the camera configured; confirm `Vision/State` reaches `TRACKING` while a
   tag is in view and `Vision/Accepted` is true.
3. Confirm `Vision/RejectionReason` reads `STALE` or `DUPLICATE_TIMESTAMP` when the camera clock or
   network is misbehaving, rather than a burst of accepted frames.
4. Wave a tag in front of the camera and confirm the reported pose moves in the expected direction
   for the alliance; MegaTag2 reports the blue-origin pose.
5. Confirm the mounting transform matches the real camera if `configureCameraPose` is enabled,
   otherwise confirm it was set in the Limelight web UI.
