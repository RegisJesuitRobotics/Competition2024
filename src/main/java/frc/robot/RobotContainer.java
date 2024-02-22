package frc.robot;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.commands.elevator.AmpPlaceCommand;
import frc.robot.commands.shooter.ShootAtAngleCommand;
import frc.robot.commands.wrist.IntakeToShooterCommand;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.telemetry.tunable.gains.TunableDouble;
import frc.robot.utils.*;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //  private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
  //  private final SwerveDriveSubsystem driveSubsystem =
  //      new SwerveDriveSubsystem(photonSubsystem::getEstimatedGlobalPose);
  private final SwerveDriveSubsystem driveSubsystem =
      new SwerveDriveSubsystem(
          (pose) -> List.of());
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final TransportSubsystem transportSubsystem = new TransportSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SlapdownSuperstructure slapdownSuperstructure = new SlapdownSuperstructure();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final SendableChooser<Command> autoCommand = new SendableChooser<>();

  private final CommandNintendoSwitchController driverController =
      new CommandNintendoSwitchController(0);
  private final CommandXboxPlaystationController operatorController =
      new CommandXboxPlaystationController(1);

  private final ListenableSendableChooser<Command> driveCommandChooser =
      new ListenableSendableChooser<>();

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureAutos();

    SmartDashboard.putData("Music", OrchestraInstance.playCommand("song1.chrp"));
    SmartDashboard.putData("Alerts", Alert.getDefaultGroup());
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
  }

  private void configureAutos() {
    autoCommand.addOption("Slapdown Q Forward", slapdownSuperstructure.getSlapdownRotationSubsystem().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption("Slapdown Q Back", slapdownSuperstructure.getSlapdownRotationSubsystem().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption("Slapdown D Forward", slapdownSuperstructure.getSlapdownRotationSubsystem().sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption("Slapdown D Back", slapdownSuperstructure.getSlapdownRotationSubsystem().sysIdDynamic(SysIdRoutine.Direction.kReverse));
  autoCommand.addOption("slapdown bottom", slapdownSuperstructure.setDownAndRunCommand());
    autoCommand.addOption("slapdown top", slapdownSuperstructure.setUpCommand());

  autoCommand.addOption("Probe Elevator", elevatorSubsystem.probeHomeCommand());
    autoCommand.addOption(
        "Wrist Q Forward", wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption(
        "Wrist Q Back", wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption(
        "Wrist D Forward", wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption(
        "Wrist D Back", wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption(
        "Wrist 5 deg",
        wristSubsystem.setPositonCommand(new Rotation2d(Units.degreesToRadians(5))));
    autoCommand.addOption(
        "Elevator Quastatic Backward",
        elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption(
        "Elevator Dynamic Forward",
        elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption(
        "Elevator Dynamic Backward",
        elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption(
        "Elevator Test Command 2 in",
        elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(2)));
    autoCommand.addOption(
        "Elevator Test Command 4 in",
        elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(4)));
    autoCommand.addOption(
        "Elevator Test Command 8 in",
        elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(8)));
    autoCommand.addOption(
        "Intake Test Command",
        intakeSubsystem.setIntakeVoltageCommand(Constants.IntakeConstants.INTAKE_VOLTAGE));
    autoCommand.addOption("Shooter Test Command 4000", shooterSubsystem.runVelocityCommand(100));
    autoCommand.addOption(
        "Shooter Quastatic Forward Command",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption("WristElevatorZero", Commands.parallel(
            wristSubsystem.setPositonCommand(Constants.WristConstants.WRIST_MIN), elevatorSubsystem.setElevatorPositionCommand(0.0)));
    autoCommand.addOption("WristElevatorNot", Commands.parallel(wristSubsystem.setPositonCommand(Rotation2d.fromDegrees(40.0)), elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(8.0))));

    autoCommand.addOption(
        "Shooter Quastatic Backward Command",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption(
        "Shooter Dynamic Forward Command",
        shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoCommand.addOption(
        "Shooter Dynamic Backward Command",
        shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoCommand.addOption(
        "Intake To Shooter",
        new IntakeToShooterCommand(
            elevatorSubsystem,
            intakeSubsystem,
            wristSubsystem,
            transportSubsystem,
            shooterSubsystem));

    SmartDashboard.putData("Auto", autoCommand);
  }

  private void configureDriverBindings() {
    configureDriving();
    driverController
            .home()
            .onTrue(
                    RaiderCommands.runOnceAllowDisable(driveSubsystem::zeroHeading)
                            .withName("ZeroHeading"));
    driverController.leftTrigger().whileTrue(transportSubsystem.setVoltageCommand(Constants.TransportConstants.TRANSPORT_VOLTAGE));
    driverController.minus().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());
    Command intakeAndFeedUntilDone = Commands.parallel(intakeSubsystem.setIntakeVoltageCommand(Constants.IntakeConstants.INTAKE_VOLTAGE), transportSubsystem.setVoltageCommand(Constants.TransportConstants.TRANSPORT_VOLTAGE)).until(transportSubsystem::atSensor).unless(transportSubsystem::atSensor);
    driverController.leftBumper().whileTrue(Commands.parallel(slapdownSuperstructure.setDownAndRunCommand(), intakeAndFeedUntilDone.asProxy(), elevatorSubsystem.setElevatorPositionCommand(0), wristSubsystem.setPositonCommand(new Rotation2d(0))));
    driverController.leftBumper().onFalse(slapdownSuperstructure.setUpCommand());
    // TODO: Speaker centric
    driverController.rightTrigger().whileTrue(Commands.none());
    // TODO: Amp auto align
    driverController.x().whileTrue(Commands.none());
    // TODO: Climb auto align
    driverController.a().whileTrue(Commands.none());
    driverController.circle().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());

  }

  private void configureOperatorBindings() {
    operatorController.triangle().onTrue(Commands.sequence(shooterSubsystem.runVelocityCommand(10000.0/60)));
    operatorController.x().onTrue(shooterSubsystem.setVoltageCommand(0));
    operatorController
        .leftTrigger()
        .onTrue(
            new IntakeToShooterCommand(
                elevatorSubsystem,
                intakeSubsystem,
                wristSubsystem,
                transportSubsystem,
                shooterSubsystem));

    //    operatorController
    //        .rightStick()
    //        .whileTrue(elevatorSubsystem.runElevatorCommand(operatorController.getRightY()));
    operatorController
        .rightTrigger()
        .onTrue(
            new ShootAtAngleCommand(
                shooterSubsystem, transportSubsystem, wristSubsystem, SHOOTING_ANGLE));
    operatorController
        .leftTrigger()
        .onTrue(
            new IntakeToShooterCommand(
                elevatorSubsystem,
                intakeSubsystem,
                wristSubsystem,
                transportSubsystem,
                shooterSubsystem));
    operatorController
        .triangle()
        .onTrue(
            new AmpPlaceCommand(
                elevatorSubsystem, wristSubsystem, shooterSubsystem, transportSubsystem));
    operatorController
        .share()
        .whileTrue(
            Commands.parallel(
                elevatorSubsystem.setElevatorPositionCommand(0),
                slapdownSuperstructure.setDownAndRunCommand(),
                intakeSubsystem.setIntakeVoltageCommand(6),
                transportSubsystem
                    .setVoltageCommand(4.0)
                    .until(transportSubsystem::atSensor)
                    .andThen(transportSubsystem.setVoltageCommand(0.0))));

    //
    // operatorController.share().onTrue(transportSubsystem.setVoltageCommand(6.0).until(transportSubsystem::atSensor));
    double rpm = 10000;
    operatorController
        .options()
        .whileTrue(
            Commands.parallel(
                elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(0)),
                shooterSubsystem.runVelocityCommand(rpm / 60),
                Commands.sequence(
                    Commands.waitUntil(shooterSubsystem::inTolerance),
                    transportSubsystem.setVoltageCommand(12.0))));
  }

  private void configureDriving() {
    TunableDouble maxTranslationSpeedPercent =
        new TunableDouble("/speed/maxTranslation", 0.9, true);
    TunableDouble maxMaxAngularSpeedPercent = new TunableDouble("/speed/maxAngular", 0.6, true);

    DoubleSupplier maxTranslationalSpeedSuppler =
        () ->
            maxTranslationSpeedPercent.get()
                * SwerveConstants.MAX_VELOCITY_METERS_SECOND
                * (driverController.leftBumper().getAsBoolean() ? 0.5 : 1);
    DoubleSupplier maxAngularSpeedSupplier =
        () -> maxMaxAngularSpeedPercent.get() * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND;

    SupplierSlewRateLimiter rotationLimiter =
        new SupplierSlewRateLimiter(
            () -> TeleopConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED);
    VectorRateLimiter vectorRateLimiter =
        new VectorRateLimiter(() -> TeleopConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED);
    Runnable resetRateLimiters =
        () -> {
          ChassisSpeeds currentSpeeds = driveSubsystem.getCurrentChassisSpeeds();
          vectorRateLimiter.reset(
              new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));
          rotationLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
        };

    driveCommandChooser.setDefaultOption(
        "Hybrid (Default to Field Relative & absolute control but use robot centric when holding button)",
        new SwerveDriveCommand(
                () ->
                    vectorRateLimiter.calculate(
                        new Translation2d(
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    driverController.getLeftY()),
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    driverController.getLeftX()))
                            .times(maxTranslationalSpeedSuppler.getAsDouble())),
                () ->
                    rotationLimiter.calculate(
                        RaiderMathUtils.deadZoneAndCubeJoystick(driverController.getRightX())
                            * maxAngularSpeedSupplier.getAsDouble()),
                driverController.rightBumper().negate(),
                driveSubsystem)
            .beforeStarting(resetRateLimiters));

    driveCommandChooser.addOption(
        "Robot Orientated",
        new SwerveDriveCommand(
                () ->
                    vectorRateLimiter.calculate(
                        new Translation2d(
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    driverController.getLeftY()),
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    driverController.getLeftX()))
                            .times(maxTranslationalSpeedSuppler.getAsDouble())),
                () ->
                    rotationLimiter.calculate(
                        RaiderMathUtils.deadZoneAndCubeJoystick(driverController.getRightX())
                            * maxAngularSpeedSupplier.getAsDouble()),
                () -> false,
                driveSubsystem)
            .beforeStarting(resetRateLimiters));

    SmartDashboard.putData("Drive Style", driveCommandChooser);
    evaluateDriveStyle(driveCommandChooser.getSelected());
    new Trigger(driveCommandChooser::hasNewValue)
        .onTrue(
            RaiderCommands.runOnceAllowDisable(
                    () -> evaluateDriveStyle(driveCommandChooser.getSelected()))
                .withName("Drive Style Checker"));
  }

  private void evaluateDriveStyle(Command newCommand) {
    Command oldCommand = driveSubsystem.getDefaultCommand();

    // Check if they are the same
    // we use the == operator instead of Command#equals() because we want to know if
    // it is the exact same object in memory
    if (newCommand == oldCommand) {
      return;
    }
    driveSubsystem.setDefaultCommand(newCommand);
    if (oldCommand != null) {
      // We have to cancel the command so the new default one will run
      oldCommand.cancel();
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand.getSelected();
  }
}
