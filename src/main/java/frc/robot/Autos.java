package frc.robot;

import static frc.robot.FieldConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.commands.drive.auto.SimpleToPointCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.telemetry.types.ProtobufTelemetryEntry;
import frc.robot.telemetry.types.StructArrayTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.utils.RaiderUtils;

public class Autos {
  private final SwerveDriveSubsystem driveSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final WristSubsystem wristSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final SlapdownSuperstructure slapdownSuperstructure;

  private final SendableChooser<Command> autoChooser;

  private final StructTelemetryEntry<Pose2d> desiredPoseTelemetryEntry = new StructTelemetryEntry<>("followPath/desiredPose", Pose2d.struct,
      MiscConstants.TUNING_MODE);
  private final StructArrayTelemetryEntry<Pose2d> trajectoryTelemetryEntry = new StructArrayTelemetryEntry<>("followPath/trajectory", Pose2d.struct,
      MiscConstants.TUNING_MODE);

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

    AutoBuilder.configureHolonomic(
        driveSubsystem::getPose,
        driveSubsystem::resetOdometry,
        driveSubsystem::getCurrentChassisSpeeds,
        (speeds) -> driveSubsystem.setChassisSpeeds(speeds, false),
        new HolonomicPathFollowerConfig(
            AutoConstants.TRANSLATION_POSITION_GAINS.createPIDConstants(),
            AutoConstants.ANGULAR_POSITION_PID_GAINS.createPIDConstants(),
            SwerveConstants.MAX_VELOCITY_METERS_SECOND,
            SwerveConstants.WHEEL_RADIUS,
            new ReplanningConfig()),
        RaiderUtils::shouldFlip,
        driveSubsystem);
    NamedCommands.registerCommand("AutoStart", autoStart());
    NamedCommands.registerCommand("ShootNote", shootNote());
    NamedCommands.registerCommand(
        "IntakeUntilNote",
        IntakingCommands.intakeUntilDetected(
            intakeSubsystem, slapdownSuperstructure, transportSubsystem));

    PathPlannerLogging.setLogActivePathCallback((path) -> {
      trajectoryTelemetryEntry.append(path.toArray(new Pose2d[0]));
    });
    PathPlannerLogging.setLogTargetPoseCallback(desiredPoseTelemetryEntry::append);

    autoChooser = AutoBuilder.buildAutoChooser("JustProbe");
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
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

  public Command autoStart() {
    if (Robot.isSimulation()) {
      return Commands.print("Probed!");
    }
    return Commands.parallel(
        elevatorSubsystem.probeHomeCommand(), slapdownSuperstructure.probeRotationHomeCommand());
  }

  private Command shootNote() {
    if (Robot.isSimulation()) {
      return Commands.print("Shooting Note!").andThen(Commands.waitSeconds(1.0));
    }
    return Commands.parallel(
            ScoringCommands.shootSetpointCloseSpeakerCommand(shooterSubsystem),
            ScoringCommands.elevatorWristCloseSpeakerCommand(elevatorSubsystem, wristSubsystem),
            Commands.parallel(
                    ScoringCommands.shooterInToleranceCommand(shooterSubsystem),
                    ScoringCommands.elevatorWristInToleranceCommand(
                        elevatorSubsystem, wristSubsystem))
                .andThen(ScoringCommands.transportCloseSpeakerCommand(transportSubsystem)))
        .until(() -> !transportSubsystem.atSensor())
        .andThen(Commands.waitSeconds(0.5));
  }
}
