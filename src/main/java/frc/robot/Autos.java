package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.telemetry.types.StructArrayTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.utils.RaiderUtils;

public class Autos {
  private final CommandSwerveDrivetrain driveSubsystem;
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
      CommandSwerveDrivetrain driveSubsystem,
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

    SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest = new SwerveRequest.ApplyChassisSpeeds();
    AutoBuilder.configureHolonomic(
        () -> driveSubsystem.getState().Pose,
        driveSubsystem::seedFieldRelative,
        () -> driveSubsystem.getState().speeds,
        (speeds) -> driveSubsystem.applyRequest(() -> applyChassisSpeedsRequest.withSpeeds(speeds).withDriveRequestType(
            DriveRequestType.Velocity)),
        new HolonomicPathFollowerConfig(
            AutoConstants.TRANSLATION_POSITION_GAINS.createPIDConstants(),
            AutoConstants.ANGULAR_POSITION_PID_GAINS.createPIDConstants(),
            AutoConstants.MAX_AUTO_VELOCITY_METERS_SECOND,
            SwerveConstants.WHEELBASE_RADIUS,
            new ReplanningConfig()),
        RaiderUtils::shouldFlip,
        driveSubsystem);
    NamedCommands.registerCommand("AutoStart", autoStart());
    NamedCommands.registerCommand("CloseShoot", shootNote());
    NamedCommands.registerCommand("IntakeUntilNote", intakeUntilNoteAndPrepareShot());
    NamedCommands.registerCommand("IntakeUntilNoteNoPrepare", intakeUntilNoteNoPrepare());
    NamedCommands.registerCommand("DynamicShoot", dynamicShoot());
    NamedCommands.registerCommand("ValidateNote", validateNote());
    NamedCommands.registerCommand("Stow", stow());

    PathPlannerLogging.setLogActivePathCallback(
        (path) -> trajectoryTelemetryEntry.append(path.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(desiredPoseTelemetryEntry::append);

    autoChooser = AutoBuilder.buildAutoChooser("JustProbe");
    if (MiscConstants.TUNING_MODE) {
      autoChooser.addOption("elevator qf", elevatorSubsystem.sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption("elevator qr", elevatorSubsystem.sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption("elevator df", elevatorSubsystem.sysIdDynamic(Direction.kForward));
      autoChooser.addOption("elevator dr", elevatorSubsystem.sysIdDynamic(Direction.kReverse));

      autoChooser.addOption("wrist qf", wristSubsystem.sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption("wrist qr", wristSubsystem.sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption("wrist df", wristSubsystem.sysIdDynamic(Direction.kForward));
      autoChooser.addOption("wrist dr", wristSubsystem.sysIdDynamic(Direction.kReverse));

//      autoChooser.addOption(
//          "drive qf", driveSubsystem.driveQuasistaticSysIDCommand(Direction.kForward));
//      autoChooser.addOption(
//          "drive qr", driveSubsystem.driveQuasistaticSysIDCommand(Direction.kReverse));
//      autoChooser.addOption(
//          "drive df", driveSubsystem.driveDynamicSysIDCommand(Direction.kForward));
//      autoChooser.addOption(
//          "drive dr", driveSubsystem.driveDynamicSysIDCommand(Direction.kReverse));
//
//      autoChooser.addOption(
//          "drive s qf", driveSubsystem.steerQuasistaticSysIDCommand(Direction.kForward));
//      autoChooser.addOption(
//          "drive s qr", driveSubsystem.steerQuasistaticSysIDCommand(Direction.kReverse));
//      autoChooser.addOption(
//          "drive s df", driveSubsystem.steerDynamicSysIDCommand(Direction.kForward));
//      autoChooser.addOption(
//          "drive s dr", driveSubsystem.steerDynamicSysIDCommand(Direction.kReverse));

      autoChooser.addOption("shooter qf", shooterSubsystem.sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption("shooter qr", shooterSubsystem.sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption("shooter df", shooterSubsystem.sysIdDynamic(Direction.kForward));
      autoChooser.addOption("shooter dr", shooterSubsystem.sysIdDynamic(Direction.kReverse));

      autoChooser.addOption(
          "slapdown rotation qf",
          slapdownSuperstructure
              .getSlapdownRotationSubsystem()
              .sysIdQuasistatic(Direction.kForward));
      autoChooser.addOption(
          "slapdown rotation qr",
          slapdownSuperstructure
              .getSlapdownRotationSubsystem()
              .sysIdQuasistatic(Direction.kReverse));
      autoChooser.addOption(
          "slapdown rotation df",
          slapdownSuperstructure.getSlapdownRotationSubsystem().sysIdDynamic(Direction.kForward));
      autoChooser.addOption(
          "slapdown rotation dr",
          slapdownSuperstructure.getSlapdownRotationSubsystem().sysIdDynamic(Direction.kReverse));

//      autoChooser.addOption(
//          "wheelRadiusCharacterizationClock",
//          new WheelRadiusCharacterization(
//              driveSubsystem, WheelRadiusCharacterization.Direction.CLOCKWISE, 0.5));
//      autoChooser.addOption(
//          "wheelRadiusCharacterizationCounterClock",
//          new WheelRadiusCharacterization(
//              driveSubsystem, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE, 0.5));
    }
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
                slapdownSuperstructure.setUpCommand(),
                elevatorWristToTag(),
                ScoringCommands.shootSetpointShootingCommand(shooterSubsystem)))
        .withName("AutoIntakeUntilNote");
  }

  private Command validateNote() {
    return Commands.waitUntil(transportSubsystem::atSensor).withTimeout(1.25);
  }

  private Command intakeUntilNoteNoPrepare() {
    if (Robot.isSimulation()) {
      return Commands.print("Intaking");
    }
    return Commands.deadline(
            IntakingCommands.intakeUntilDetected(
                intakeSubsystem, slapdownSuperstructure, transportSubsystem),
            ElevatorWristCommands.elevatorWristIntakePosition(elevatorSubsystem, wristSubsystem))
        .withName("AutoIntakeUntilNoteNoPrepare");
  }

  public Command autoStart() {
    if (Robot.isSimulation()) {
      return Commands.print("Probed!");
    }
    return Commands.parallel(
            elevatorSubsystem.probeHomeCommand(),
            slapdownSuperstructure.getSlapdownRotationSubsystem().probeHomeCommand())
        .withName("AutoStart");
  }

  private Command shootNote() {
    if (Robot.isSimulation()) {
      return Commands.print("Shooting Note!").andThen(Commands.waitSeconds(0.5));
    }
    return Commands.deadline(
            Commands.sequence(
                shooterAndElevatorWristInToleranceCommand(),
                ScoringCommands.transportToShooterCommand(transportSubsystem)
                    .until(() -> !transportSubsystem.atSensor())),
            ScoringCommands.shootSetpointShootingCommand(shooterSubsystem),
            ElevatorWristCommands.elevatorWristCloseSpeakerCommand(
                elevatorSubsystem, wristSubsystem))
        .andThen(transportSubsystem.stopCommand())
        .withName("AutoShootNote");
  }

  private Command stow() {
    return Commands.parallel(
        ElevatorWristCommands.elevatorWristZeroCommand(elevatorSubsystem, wristSubsystem),
        slapdownSuperstructure.setUpCommand(),
        ScoringCommands.shootSetpointIdleCommand(shooterSubsystem)
    );
  }

  private Command dynamicShoot() {
    return Commands.deadline(
            Commands.sequence(
                shooterAndElevatorWristInToleranceCommand(),
                ScoringCommands.transportToShooterCommand(transportSubsystem)
                    .until(() -> !transportSubsystem.atSensor())),
            ScoringCommands.shootSetpointShootingCommand(shooterSubsystem),
            elevatorWristToTag())
        .andThen(transportSubsystem.stopCommand())
        .withName("AutoShootDynamicNote");
  }

  private Command shooterAndElevatorWristInToleranceCommand() {
    return Commands.parallel(
            ScoringCommands.shooterInToleranceCommand(shooterSubsystem),
            ElevatorWristCommands.elevatorWristInToleranceCommand(
                elevatorSubsystem, wristSubsystem))
        .withName("ShooterAndElevatorWristInTolerance");
  }

  private Command elevatorWristToTag() {
    return ElevatorWristCommands.elevatorWristDynamicCommand(
        () -> SetpointConstants.REGULAR_SHOT_ELEVATOR_HEIGHT_METERS,
        () -> SetpointConstants.REGULAR_SHOT_WRIST_SETPOINT_TABLE.get(getCameraDistanceFromTag()),
        elevatorSubsystem,
        wristSubsystem);
  }

  private double getCameraDistanceFromTag() {
    int desiredTag = RaiderUtils.shouldFlip() ? 4 : 7;
    return driveSubsystem
        .getState()
        .Pose
        .getTranslation()
        .plus(VisionConstants.ROBOT_TO_CAM.getTranslation().toTranslation2d())
        .getDistance(
            FieldConstants.aprilTags
                .getTagPose(desiredTag)
                .get()
                .getTranslation()
                .toTranslation2d());
  }
}
