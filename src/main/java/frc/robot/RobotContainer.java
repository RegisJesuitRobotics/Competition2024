package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.MiscCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.photon.PhotonSubsystem;
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
  private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
    private final SwerveDriveSubsystem driveSubsystem =
        new SwerveDriveSubsystem(photonSubsystem::getEstimatedGlobalPose);
//  private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem((pose) -> List.of());
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final TransportSubsystem transportSubsystem = new TransportSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SlapdownSuperstructure slapdownSuperstructure = new SlapdownSuperstructure();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final Autos autos =
      new Autos(
          driveSubsystem,
          elevatorSubsystem,
          shooterSubsystem,
          wristSubsystem,
          transportSubsystem,
          intakeSubsystem,
          slapdownSuperstructure);

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

    SmartDashboard.putData("Music", OrchestraInstance.playCommand("song10.chrp"));
    SmartDashboard.putData("Alerts", Alert.getDefaultGroup());
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
  }

  private void configureAutos() {
    // autoCommand.addOption("Test Path", autos.testPathAuth());
    // autoCommand.addOption(
    //     "Drive SysID QF",
    // driveSubsystem.quasistaticSysIDCommand(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Drive SysID QR",
    // driveSubsystem.quasistaticSysIDCommand(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Drive SysID DF", driveSubsystem.dynamicSysIDCommand(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Drive SysID DR", driveSubsystem.dynamicSysIDCommand(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Slapdown SysID QF",
    //     slapdownSuperstructure
    //         .getSlapdownRotationSubsystem()
    //         .sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Slapdown SysID QR",
    //     slapdownSuperstructure
    //         .getSlapdownRotationSubsystem()
    //         .sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Slapdown SysID DF",
    //     slapdownSuperstructure
    //         .getSlapdownRotationSubsystem()
    //         .sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Slapdown SysID DR",
    //     slapdownSuperstructure
    //         .getSlapdownRotationSubsystem()
    //         .sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption("Probe Elevator", elevatorSubsystem.probeHomeCommand());
    //     autoCommand.addOption(
    //         "Wrist SysID QF", wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Wrist SysID QR", wristSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Wrist SysID DF", wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Wrist SysID DR", wristSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Elevator SysID QF",
    // elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Elevator SysID QR",
    // elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Elevator SysID DF", elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Elevator SysID DR", elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Shooter SysID QF", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Shooter SysID QR", shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoCommand.addOption(
    //     "Shooter SysID DF", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoCommand.addOption(
    //     "Shooter SysID DR", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto", autos.getAutoChooser());
  }

  private void configureDriverBindings() {
    configureDriving();
    driverController
        .home()
        .onTrue(
            RaiderCommands.runOnceAllowDisable(driveSubsystem::zeroHeading)
                .withName("ZeroHeading"));
    driverController
        .leftTrigger()
        .whileTrue(
            transportSubsystem.setVoltageCommand(
                Constants.TransportConstants.TRANSPORT_CLOSE_SPEAKER_VOLTAGE));
    driverController.minus().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());
    driverController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                IntakingCommands.intakeUntilDetected(
                    intakeSubsystem, slapdownSuperstructure, transportSubsystem),
                elevatorSubsystem.setElevatorPositionCommand(Units.inchesToMeters(2.5)),
                wristSubsystem.setPositonCommand(new Rotation2d(0))));
    driverController.rightTrigger().onFalse(slapdownSuperstructure.setUpCommand());
    driverController.circle().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());
  }

  private void configureOperatorBindings() {
    operatorController
        .square()
        .onTrue(
            Commands.parallel(
                ScoringCommands.shootSetpointAmpCommand(shooterSubsystem),
                transportSubsystem.setVoltageCommand(12)));
    operatorController
        .triangle()
        .onTrue(
            Commands.parallel(
                ScoringCommands.shootSetpointCloseSpeakerCommand(shooterSubsystem),
                Commands.waitUntil(shooterSubsystem::inTolerance)
                    .andThen(MiscCommands.rumbleHIDCommand(operatorController.getHID()))));
    operatorController.circle().onTrue(ScoringCommands.shootSetpointIdleCommand(shooterSubsystem));
    operatorController.x().onTrue(ScoringCommands.shootSetpointZeroCommand(shooterSubsystem));

    operatorController
        .povUp()
        .onTrue(
            ScoringCommands.elevatorWristCloseSpeakerCommand(elevatorSubsystem, wristSubsystem));
    operatorController
        .povDown()
        .onTrue(ScoringCommands.elevatorWristAmpCommand(elevatorSubsystem, wristSubsystem));
    operatorController
        .leftTrigger()
        .onTrue(ScoringCommands.elevatorWristZeroCommand(elevatorSubsystem, wristSubsystem));

    operatorController
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  climberSubsystem.setVoltage(operatorController.getLeftX() * 10);
                },
                () -> climberSubsystem.setVoltage(0.0),
                climberSubsystem));
  }

  private void configureDriving() {
    TunableDouble maxTranslationSpeedPercent =
        new TunableDouble("/speed/maxTranslation", 0.95, true);
    TunableDouble maxMaxAngularSpeedPercent = new TunableDouble("/speed/maxAngular", 0.6, true);

    DoubleSupplier maxTranslationalSpeedSuppler =
        () -> maxTranslationSpeedPercent.get() * SwerveConstants.MAX_VELOCITY_METERS_SECOND;
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
                                    -driverController.getLeftY()),
                                RaiderMathUtils.deadZoneAndCubeJoystick(
                                    -driverController.getLeftX()))
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
    return autos.getAutoChooser().getSelected();
  }
}
