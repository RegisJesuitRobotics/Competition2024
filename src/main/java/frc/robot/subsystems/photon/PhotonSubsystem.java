package frc.robot.subsystems.photon;

import static frc.robot.Constants.ROBOT_TO_CAM;
import static frc.robot.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.FieldConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.StructArrayTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.RaiderUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonSubsystem extends SubsystemBase {
  private final AprilTagFieldLayout fieldLayout;
  PhotonPoseEstimator poseEstimator;

  private final StructArrayTelemetryEntry<Pose3d> estimatedPoseEntries =
      new StructArrayTelemetryEntry<>(
          "/photon/estimatedPoses", Pose3d.struct, MiscConstants.TUNING_MODE);
  private final StructArrayTelemetryEntry<Pose3d> visionTargetEntries =
      new StructArrayTelemetryEntry<>("/photon/targets", Pose3d.struct, MiscConstants.TUNING_MODE);

  private final DoubleTelemetryEntry distanceEntry = new DoubleTelemetryEntry("/photon/distance", true);

  private final PhotonCamera camera = new PhotonCamera("MainCamera");
  private final Alert cameraNotConnectedAlert =
      new Alert("AprilTag Camera is Not Powered or Not Connected", AlertType.ERROR);

  public PhotonSubsystem() {
    fieldLayout = FieldConstants.aprilTags;
    fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, ROBOT_TO_CAM);

    estimatedPoseEntries.append(new Pose3d[0]);
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

  public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
    List<Pose3d> targetPoses = new ArrayList<>();

    PhotonPipelineResult result = camera.getLatestResult();

    // Remove bad tags if only one, also add to our array
    for (int j = result.targets.size() - 1; j >= 0; j--) {
      PhotonTrackedTarget target = result.targets.get(j);
      boolean shouldUse =
          (result.targets.get(j).getPoseAmbiguity() < VisionConstants.POSE_AMBIGUITY_CUTOFF
                  || result.targets.size() > 1)
              && result.targets.get(j).getBestCameraToTarget().getTranslation().getNorm()
                  < VisionConstants.DISTANCE_CUTOFF;

      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent() && shouldUse) {
        Pose3d tagPose = fieldLayout.getTagPose(target.getFiducialId()).get();

        targetPoses.add(tagPose);
      }
      if (!shouldUse) {
        result.targets.remove(j);
      }
    }

    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
    if (estimatedRobotPose.isPresent()) {
      estimatedPoseEntries.append(new Pose3d[] {estimatedRobotPose.get().estimatedPose});
      updatedPoses.add(estimatedRobotPose.get());
    } else {
      estimatedPoseEntries.append(new Pose3d[] {});
    }

    visionTargetEntries.append(targetPoses.toArray(new Pose3d[0]));

    return updatedPoses;
  }

  @Override
  public void periodic() {
    boolean allCamerasConnected = camera.isConnected();
    getEstimatedGlobalPose(new Pose2d());
    if (getDistanceSpeaker().isPresent()) {
      distanceEntry.append(getDistanceSpeaker().getAsDouble());
    }else{
      distanceEntry.append(-1);
    }
    cameraNotConnectedAlert.set(!allCamerasConnected);
  }
}
