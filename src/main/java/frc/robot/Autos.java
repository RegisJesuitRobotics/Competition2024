package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
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

  private final StructTelemetryEntry<Pose2d> desiredPoseTelemetryEntry =
      new StructTelemetryEntry<>(
          "followPath/desiredPose", Pose2d.struct, MiscConstants.TUNING_MODE);
  private final StructArrayTelemetryEntry<Pose2d> trajectoryTelemetryEntry =
      new StructArrayTelemetryEntry<>(
          "followPath/trajectory", Pose2d.struct, MiscConstants.TUNING_MODE);

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
    NamedCommands.registerCommand("IntakeUntilNote", intakeUntilNoteAndPrepareShot());

    PathPlannerLogging.setLogActivePathCallback(
        (path) -> trajectoryTelemetryEntry.append(path.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          driveSubsystem.resetOdometry(pose);
          desiredPoseTelemetryEntry.append(pose);
        });

    autoChooser = AutoBuilder.buildAutoChooser("JustProbe");
    autoChooser.addOption("shooter qf", shooterSubsystem.sysIdQuasistatic(Direction.kForward));
    autoChooser.addOption("shooter qr", shooterSubsystem.sysIdQuasistatic(Direction.kReverse));
    autoChooser.addOption("shooter df", shooterSubsystem.sysIdDynamic(Direction.kForward));
    autoChooser.addOption("shooter dr", shooterSubsystem.sysIdDynamic(Direction.kReverse));

    autoChooser.addOption("elevator qf", elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
    autoChooser.addOption("elevator qr", elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
    autoChooser.addOption("elevator df", elevatorSubsystem.sysIdDynamic(Direction.kForward));
    autoChooser.addOption("elevator dr", elevatorSubsystem.sysIdDynamic(Direction.kReverse));

    autoChooser.addOption(
        "drive qf", driveSubsystem.driveQuasistaticSysIDCommand(Direction.kForward));
    autoChooser.addOption(
        "drive qr", driveSubsystem.driveQuasistaticSysIDCommand(Direction.kReverse));
    autoChooser.addOption("drive df", driveSubsystem.driveDynamicSysIDCommand(Direction.kForward));
    autoChooser.addOption("drive dr", driveSubsystem.driveDynamicSysIDCommand(Direction.kReverse));

    autoChooser.addOption(
        "drive s qf", driveSubsystem.steerQuasistaticSysIDCommand(Direction.kForward));
    autoChooser.addOption(
        "drive s qr", driveSubsystem.steerQuasistaticSysIDCommand(Direction.kReverse));
    autoChooser.addOption(
        "drive s df", driveSubsystem.steerDynamicSysIDCommand(Direction.kForward));
    autoChooser.addOption(
        "drive s dr", driveSubsystem.steerDynamicSysIDCommand(Direction.kReverse));
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  private Command intakeUntilNoteAndPrepareShot() {
    if (Robot.isSimulation()) {
      return Commands.print("Intaking");
    }
    return Commands.deadline(
            IntakingCommands.intakeUntilDetected(
                intakeSubsystem, slapdownSuperstructure, transportSubsystem),
            ElevatorWristCommands.elevatorWristIntakePosition(elevatorSubsystem, wristSubsystem))
        .andThen(
            Commands.parallel(
                ElevatorWristCommands.elevatorWristCloseSpeakerCommand(
                    elevatorSubsystem, wristSubsystem),
                ScoringCommands.shootSetpointCloseSpeakerCommand(shooterSubsystem)));
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
      return Commands.print("Shooting Note!").andThen(Commands.waitSeconds(0.5));
    }
    return Commands.deadline(
            Commands.sequence(
                    shooterAndElevatorWristInToleranceCommand(),
                    ScoringCommands.transportCloseSpeakerCommand(transportSubsystem))
                .until(() -> !transportSubsystem.atSensor())
                .andThen(
                    Commands.waitSeconds(0.1),
                    ScoringCommands.shootSetpointCloseSpeakerCommand(shooterSubsystem),
                    ElevatorWristCommands.elevatorWristCloseSpeakerCommand(
                        elevatorSubsystem, wristSubsystem)))
        .andThen(
            Commands.runOnce(
                () -> transportSubsystem.runShooterTransportVoltage(0.0), transportSubsystem));
  }

  private Command shooterAndElevatorWristInToleranceCommand() {
    return Commands.parallel(
        ScoringCommands.shooterInToleranceCommand(shooterSubsystem),
        ElevatorWristCommands.elevatorWristInToleranceCommand(elevatorSubsystem, wristSubsystem));
  }
}
