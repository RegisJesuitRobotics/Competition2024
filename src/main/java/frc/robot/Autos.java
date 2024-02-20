package frc.robot;

import static frc.robot.FieldConstants.*;
import static frc.robot.FieldConstants.Stage.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class Autos {

//  public static Command nearestClimberCommand(Alliance alliance, SwerveDriveSubsystem swerve) {
//    Pose2d stagingLoc;
//    if (alliance == Alliance.Red) {
//      stagingLoc = swerve.getPose().nearest(redStagingLocations);
//    } else {
//      stagingLoc = swerve.getPose().nearest(blueStagingLocations);
//    }
//    return new SimpleToPointCommand(stagingLoc, swerve);
//  }

  public static Command nearestAmpCommand(Alliance alliance, SwerveDriveSubsystem swerve) {
    Pose2d ampLoc;
    // TODO; THIS
    if (alliance == Alliance.Red) {
      ampLoc = new Pose2d(ampCenterRed, new Rotation2d(0, 0));
    } else {
      ampLoc = new Pose2d(ampCenterBlue, new Rotation2d(0, 0));
    }
    return new SimpleToPointCommand(ampLoc, swerve);
  }
}
