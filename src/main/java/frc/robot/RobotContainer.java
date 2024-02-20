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
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
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
          (pose) -> {
            return List.of();
          });
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final TransportSubsystem transportSubsystem = new TransportSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SlapdownSubsystem slapdownSubsystem = new SlapdownSubsystem();

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
    autoCommand.addOption(
        "Elevator Quastatic Forward",
        elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
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
    autoCommand.addOption("Slap Feed", slapdownSubsystem.setFeederVoltageCommand(5));
    autoCommand.addOption(
        "Intake feed",
        Commands.parallel(
            slapdownSubsystem.setFeederVoltageCommand(5),
            intakeSubsystem.setIntakeVoltageCommand(Constants.IntakeConstants.INTAKE_VOLTAGE)));
    autoCommand.addOption(
        "Shooter Quastatic Forward Command",
        shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
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
    driverController.minus().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());

    //    driverController
    //        .povLeft()
    //        .and(getDistanceToStaging(DriverStation.getAlliance().get(), driveSubsystem))
    //        .onTrue(nearestClimberCommand(DriverStation.getAlliance().get(), driveSubsystem));
    //    driverController
    //        .povRight()
    //        .onTrue(nearestAmpCommand(DriverStation.getAlliance().get(), driveSubsystem));
    driverController
        .povDown()
        .toggleOnTrue(slapdownSubsystem.setRotationGoalCommand(new Rotation2d(0)));
    driverController
        .povDown()
        .toggleOnTrue(slapdownSubsystem.setFeederVoltageCommand(4)); // TODO: THIS
    driverController
        .povDown()
        .toggleOnFalse(
            slapdownSubsystem.setRotationGoalCommand(new Rotation2d(Units.degreesToRadians(90))));
    driverController.povDown().toggleOnFalse(slapdownSubsystem.setFeederVoltageCommand(0));
    driverController.a().onTrue(intakeSubsystem.checkIntakeCommand());
  }

  private void configureOperatorBindings() {
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
                                    -1 * driverController.getLeftY()),
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    -1 * driverController.getLeftX()))
                            .times(maxTranslationalSpeedSuppler.getAsDouble())),
                () ->
                    rotationLimiter.calculate(
                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
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
                                    -1 * driverController.getLeftY()),
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    -1 * driverController.getLeftX()))
                            .times(maxTranslationalSpeedSuppler.getAsDouble())),
                () ->
                    rotationLimiter.calculate(
                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
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
