package frc.robot.commands.drive.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.StructArrayTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import java.util.function.Supplier;

public class FollowPathCommand extends Command {
  private static final StructTelemetryEntry<Pose2d> desiredPoseEntry =
      new StructTelemetryEntry<>(
          "/followPath/desiredPose", Pose2d.struct, MiscConstants.TUNING_MODE);
  private static final StructArrayTelemetryEntry<Pose2d> trajectoryEntry =
      new StructArrayTelemetryEntry<>(
          "/followPath/trajectory", Pose2d.struct, MiscConstants.TUNING_MODE);

  private final SwerveDriveSubsystem driveSubsystem;
  private final Supplier<ChoreoTrajectory> pathSupplier;

  private final ChoreoControlFunction controller =
      Choreo.choreoSwerveController(
          new TunableTelemetryPIDController(
              "/followPath/xController", AutoConstants.TRANSLATION_POSITION_GAINS),
          new TunableTelemetryPIDController(
              "/followPath/yController", AutoConstants.TRANSLATION_POSITION_GAINS),
          new TunableTelemetryPIDController(
              "/followPath/rotationController", AutoConstants.ANGULAR_POSITION_PID_GAINS));

  private final Timer timer = new Timer();
  private ChoreoTrajectory currentPath;

  public FollowPathCommand(ChoreoTrajectory path, SwerveDriveSubsystem driveSubsystem) {
    this(path, true, driveSubsystem);
  }

  public FollowPathCommand(
      ChoreoTrajectory path, boolean shouldFlipIfRed, SwerveDriveSubsystem driveSubsystem) {
    this(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (shouldFlipIfRed && (alliance.isPresent() && alliance.get() == Alliance.Red)) {
            return path.flipped();
          }
          return path;
        },
        driveSubsystem);
  }

  public FollowPathCommand(
      Supplier<ChoreoTrajectory> pathSupplier, SwerveDriveSubsystem driveSubsystem) {
    this.pathSupplier = pathSupplier;
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    currentPath = pathSupplier.get();

    // TODO: Reset odometry?

    trajectoryEntry.append(currentPath.getPoses());
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double currentTime = timer.get();
    Pose2d currentPose = driveSubsystem.getPose();

    ChoreoTrajectoryState desiredState = currentPath.sample(currentTime);
    ChassisSpeeds chassisSpeeds = controller.apply(currentPose, desiredState);

    desiredPoseEntry.append(desiredState.getPose());
    driveSubsystem.setChassisSpeeds(chassisSpeeds, false);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopMovement();
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(currentPath.getTotalTime());
  }
}
