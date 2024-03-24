package frc.robot.subsystems.photon;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.FieldConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.RaiderUtils;
import java.util.OptionalDouble;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSubsystem extends SubsystemBase {
  private final AprilTagFieldLayout fieldLayout;

  private final DoubleTelemetryEntry distanceEntry =
      new DoubleTelemetryEntry("/photon/distance", true);
  private final DoubleTelemetryEntry offsetEntry =
      new DoubleTelemetryEntry("/photon/rotationOffset", true);

  private final DoubleTelemetryEntry photonWristSetpoint =
      new DoubleTelemetryEntry("/photon/wristSetpoint", true);

  private final PhotonCamera camera = new PhotonCamera("MainCamera");
  private final Alert cameraNotConnectedAlert =
      new Alert("AprilTag Camera is Not Powered or Not Connected", AlertType.ERROR);

  public PhotonSubsystem() {
    fieldLayout = FieldConstants.aprilTags;
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }

  public OptionalDouble getDistanceSpeaker() {
    int desiredTag = RaiderUtils.shouldFlip() ? 4 : 7;

    PhotonPipelineResult result = camera.getLatestResult();

    for (PhotonTrackedTarget target : result.targets) {

      if (target.getFiducialId() == desiredTag) {
        return OptionalDouble.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                ROBOT_TO_CAM.getZ(),
                fieldLayout.getTagPose(desiredTag).get().getZ(),
                -ROBOT_TO_CAM.getRotation().getY(),
                Units.degreesToRadians(target.getPitch())));
      }
    }
    return OptionalDouble.empty();
  }

  public OptionalDouble getOffsetRadiansSpeaker() {
    int desiredTag = RaiderUtils.shouldFlip() ? 4 : 7;

    PhotonPipelineResult result = camera.getLatestResult();

    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == desiredTag) {
        return OptionalDouble.of(Units.degreesToRadians(target.getYaw()));
      }
    }
    return OptionalDouble.empty();
  }

  @Override
  public void periodic() {
    boolean allCamerasConnected = camera.isConnected();
    cameraNotConnectedAlert.set(!allCamerasConnected);

    OptionalDouble distance = getDistanceSpeaker();
    if (distance.isPresent()) {
      distanceEntry.append(distance.getAsDouble());
      photonWristSetpoint.append(
          SetpointConstants.REGULAR_SHOT_WRIST_SETPOINT_TABLE.get(distance.getAsDouble())
              + WristConstants.WRIST_TO_SHOOTER);
    } else {
      distanceEntry.append(Double.NaN);
      photonWristSetpoint.append(Double.NaN);
    }

    OptionalDouble offset = getOffsetRadiansSpeaker();
    if (offset.isPresent()) {
      offsetEntry.append(offset.getAsDouble());
    } else {
      offsetEntry.append(Double.NaN);
    }
  }
}
