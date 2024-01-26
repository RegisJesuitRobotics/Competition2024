package frc.robot.subsystems.photon;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAM;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.rich.Pose3dArrayEntry;
import frc.robot.telemetry.types.rich.Pose3dEntry;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import java.io.IOException;
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
  private final List<Pose3dEntry> estimatedPoseEntries = new ArrayList<>();
  private final Pose3dArrayEntry visionTargetEntries =
      new Pose3dArrayEntry("/photon/targets", MiscConstants.TUNING_MODE);
  private final Pose3dArrayEntry unusedVisionTargetEntries =
      new Pose3dArrayEntry("/photon/unusedTargets", MiscConstants.TUNING_MODE);
  private final PhotonCamera camera = new PhotonCamera("Camera");
  private final Alert cameraNotConnectedAlert =
      new Alert("AprilTag Camera is Not Powered or Not Connected", AlertType.ERROR);
  private double[] temp = new double[7];
  private Pose3dEntry poseEntry = new Pose3dEntry("/vision/estimatedPose", false);

  public PhotonSubsystem() {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, ROBOT_TO_CAM);

    estimatedPoseEntries.add(new Pose3dEntry("/photon/estimatedPoses/", MiscConstants.TUNING_MODE));
  }

  public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //        Robot.startWNode("PhotonSubsystem#getEstimatedGlobalPose");
    List<EstimatedRobotPose> updatedPoses = new ArrayList<>();
    List<Pose3d> targetPoses = new ArrayList<>();
    List<Pose3d> unusedTargetPoses = new ArrayList<>();

    PhotonPipelineResult result = camera.getLatestResult();

    // Remove bad tags if only one, also add to our array
    for (int j = result.targets.size() - 1; j >= 0; j--) {
      PhotonTrackedTarget target = result.targets.get(j);

      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose3d tagPose = fieldLayout.getTagPose(target.getFiducialId()).get();

          targetPoses.add(tagPose);

      }

        result.targets.remove(j);

    }

    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
    poseEntry.append(estimatedRobotPose.get().estimatedPose);
    if (estimatedRobotPose.isPresent()) {
      estimatedPoseEntries.add(poseEntry);
      updatedPoses.add(estimatedRobotPose.get());
    }

    visionTargetEntries.append(targetPoses);
    unusedVisionTargetEntries.append(unusedTargetPoses);

    //        Robot.endWNode();
    return updatedPoses;
  }

  @Override
  public void periodic() {
    //        Robot.startWNode("PhotonSubsystem#periodic");
    boolean allCamerasConnected = true;

    allCamerasConnected &= camera.isConnected();

    cameraNotConnectedAlert.set(!allCamerasConnected);
    //        Robot.endWNode();
  }
}
