package frc.robot;

import static frc.robot.FieldConstants.Stage.*;
import static frc.robot.FieldConstants.StagingLocations.stagingThreshold;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import java.util.function.BooleanSupplier;

public class Autos {

  public static Command nearestClimberCommand(Alliance alliance, SwerveDriveSubsystem swerve) {
    Pose2d stagingLoc;
    if (alliance == Alliance.Red) {
      stagingLoc = swerve.getPose().nearest(redStagingLocations);
    } else {
      stagingLoc = swerve.getPose().nearest(blueStagingLocations);
    }
    return new SimpleToPointCommand(stagingLoc, swerve);
  }

  public static BooleanSupplier getDistanceToStaging(
      Alliance alliance, SwerveDriveSubsystem swerve) {
    BooleanSupplier inThreshold;
    if (alliance == Alliance.Red)
      return inThreshold =
          () -> {
            return swerve
                    .getPose()
                    .nearest(redStagingLocations)
                    .getTranslation()
                    .getDistance(swerve.getPose().getTranslation())
                < stagingThreshold;
          };
    else {
      return inThreshold =
          () -> {
            return swerve
                    .getPose()
                    .nearest(blueStagingLocations)
                    .getTranslation()
                    .getDistance(swerve.getPose().getTranslation())
                < stagingThreshold;
          };
    }
  }
}
