package frc.robot;

import static frc.robot.FieldConstants.*;
import static frc.robot.FieldConstants.Stage.*;

import com.choreo.lib.Choreo;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.commands.drive.auto.FollowPathCommand;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class Autos {
  private final SwerveDriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final WristSubsystem wristSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SlapdownSuperstructure slapdownSuperstructure;

  public Autos(
      SwerveDriveSubsystem driveSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ShooterSubsystem shooterSubsystem,
      WristSubsystem wristSubsystem,
      TransportSubsystem transportSubsystem,
      IntakeSubsystem intakeSubsystem,
      SlapdownSuperstructure slapdownSuperstructure) {
    this.driveSubsystem = driveSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.transportSubsystem = transportSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.slapdownSuperstructure = slapdownSuperstructure;
  }

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

  private Command autoStart() {
    return Commands.parallel(
        elevatorSubsystem.probeHomeCommand(), slapdownSuperstructure.probeRotationHomeCommand());
  }

  private Command shootNote() {
    return Commands.parallel(
        ScoringCommands.shootSetpointCloseSpeakerCommand(shooterSubsystem),
        ScoringCommands.elevatorWristCloseSpeakerCommand(elevatorSubsystem, wristSubsystem),
        Commands.parallel(
                ScoringCommands.shooterInToleranceCommand(shooterSubsystem),
                ScoringCommands.elevatorWristInToleranceCommand(elevatorSubsystem, wristSubsystem))
            .andThen(ScoringCommands.transportCloseSpeakerCommand(transportSubsystem)));
  }

  public Command centerSpeakerCloseFourPieceAuto() {
    return Commands.sequence(
        autoStart(),
        shootNote(),
        centerSpeakerCloseRightNote(),
        centerSpeakerCloseMidNote(),
        centerSpeakerCloseLeftNote());
  }

  public Command centerSpeakerCloseRightMidThreePieceAuto() {
    return Commands.sequence(
        autoStart(), shootNote(), centerSpeakerCloseRightNote(), centerSpeakerCloseMidNote());
  }

  public Command centerSpeakerCloseLeftMidThreePieceAuto() {
    return Commands.sequence(
        autoStart(), shootNote(), centerSpeakerCloseLeftNote(), centerSpeakerCloseMidNote());
  }

  public Command centerSpeakerCloseRightTwoPieceAuto() {
    return Commands.sequence(autoStart(), shootNote(), centerSpeakerCloseRightNote());
  }

  public Command centerSpeakerCloseMidTwoPieceAuto() {
    return Commands.sequence(autoStart(), shootNote(), centerSpeakerCloseMidNote());
  }

  public Command centerSpeakerCloseLeftTwoPieceAuto() {
    return Commands.sequence(autoStart(), shootNote(), centerSpeakerCloseLeftNote());
  }

  public Command centerSpeakerOnePieceAuto() {
    return Commands.sequence(autoStart(), shootNote());
  }

  private Command centerSpeakerCloseMidNote() {
    return Commands.sequence(
        Commands.parallel(
            followPathCommand("CenterSpeakerCloseMidNote", driveSubsystem),
            IntakingCommands.intakeUntilDetected(
                intakeSubsystem, slapdownSuperstructure, transportSubsystem)),
        shootNote());
  }

  private Command centerSpeakerCloseRightNote() {
    return Commands.sequence(
        Commands.parallel(
            followPathCommand("CenterSpeakerCloseRightNote", driveSubsystem),
            IntakingCommands.intakeUntilDetected(
                intakeSubsystem, slapdownSuperstructure, transportSubsystem)),
        shootNote());
  }

  private Command centerSpeakerCloseLeftNote() {
    return Commands.sequence(
        Commands.parallel(
            followPathCommand("CenterSpeakerCloseLeftNote", driveSubsystem),
            IntakingCommands.intakeUntilDetected(
                intakeSubsystem, slapdownSuperstructure, transportSubsystem)),
        shootNote());
  }

  public Command followPathCommand(String path, SwerveDriveSubsystem driveSubsystem) {
    return new FollowPathCommand(Choreo.getTrajectory(path), driveSubsystem);
  }
}
