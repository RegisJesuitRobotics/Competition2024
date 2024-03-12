package frc.robot.subsystems.photon;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.FieldConstants;
import frc.robot.telemetry.types.StructArrayTelemetryEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
    List<Pose3d> targetPoses = new ArrayList<>();

    PhotonPipelineResult result = camera.getLatestResult();

    // Remove bad tags if only one, also add to our array
    for (int j = result.targets.size() - 1; j >= 0; j--) {
      PhotonTrackedTarget target = result.targets.get(j);
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose3d tagPose = fieldLayout.getTagPose(target.getFiducialId()).get();

        targetPoses.add(tagPose);
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

    cameraNotConnectedAlert.set(!allCamerasConnected);
  }
}
