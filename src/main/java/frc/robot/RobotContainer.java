package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.IntakingCommands;
import frc.robot.commands.MiscCommands;
import frc.robot.commands.ScoringCommands;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.commands.led.LEDStateMachineCommand;
import frc.robot.commands.led.LEDStateMachineCommand.LEDState;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.photon.PhotonSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.tunable.gains.TunableDouble;
import frc.robot.utils.*;
import frc.robot.utils.led.AlternatePattern;
import frc.robot.utils.led.SlidePattern;
import frc.robot.utils.led.SolidPattern;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final PhotonSubsystem photonSubsystem = new PhotonSubsystem();
  //    private final SwerveDriveSubsystem driveSubsystem =
  //        new SwerveDriveSubsystem(photonSubsystem::getEstimatedGlobalPose);
  private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem((pose) -> List.of());
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final TransportSubsystem transportSubsystem = new TransportSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SlapdownSuperstructure slapdownSuperstructure = new SlapdownSuperstructure();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

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

  private final AtomicBoolean signalHumanPlayer = new AtomicBoolean(false);

  private final TunableTelemetryPIDController snapController =
      new TunableTelemetryPIDController("/snap/controller", AutoConstants.SNAP_POSITION_PID_GAINS);

  public RobotContainer() {
    configureDriverBindings();
    configureOperatorBindings();
    configureLEDs();

    SmartDashboard.putData("Auto", autos.getAutoChooser());
    SmartDashboard.putData("Music", OrchestraInstance.playCommand("song10.chrp"));
    SmartDashboard.putData("Alerts", Alert.getDefaultGroup());
    SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
  }

  private void configureLEDs() {
    // Default state for homing lights
    ledSubsystem.setStatusLight(0, Color.kRed);
    ledSubsystem.setStatusLight(1, Color.kRed);

    // Set homing lights to state
    new Trigger(slapdownSuperstructure.getSlapdownRotationSubsystem()::isHomed)
        .onFalse(
            Commands.runOnce(() -> ledSubsystem.setStatusLight(0, Color.kRed))
                .ignoringDisable(true)
                .withName("SlapdownLEDStatusFalse"))
        .onTrue(
            Commands.runOnce(() -> ledSubsystem.setStatusLight(0, Color.kGreen))
                .ignoringDisable(true)
                .withName("SlapdownLEDStatusTrue"));
    new Trigger(elevatorSubsystem::isHomed)
        .onFalse(
            Commands.runOnce(() -> ledSubsystem.setStatusLight(1, Color.kRed))
                .ignoringDisable(true)
                .withName("ElevatorLEDStatusFalse"))
        .onTrue(
            Commands.runOnce(() -> ledSubsystem.setStatusLight(1, Color.kGreen))
                .ignoringDisable(true)
                .withName("ElevatorLEDStatusTrue"));

    AtomicBoolean transportSensorBlink = new AtomicBoolean();
    new Trigger(transportSubsystem::atSensor)
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> transportSensorBlink.set(true)),
                    Commands.waitSeconds(1),
                    Commands.runOnce(() -> transportSensorBlink.set(false)))
                .withName("TransportSensorBlink"));

    List<LEDState> ledStates =
        List.of(
            // Signal human player
            new LEDState(
                signalHumanPlayer::get, new AlternatePattern(0.5, Color.kOrange, Color.kBlack)),
            // Blink after intake
            new LEDState(
                transportSensorBlink::get,
                new AlternatePattern(0.25, Color.kAliceBlue, Color.kBlack)),
            // Red blink if we have any faults
            new LEDState(
                () -> Alert.getDefaultGroup().hasAnyErrors(),
                new AlternatePattern(2.0, Color.kRed, Color.kBlack)),
            // Purple slide if ready to shoot
            new LEDState(
                () -> {
                  // Is snaping
                  return DriverStation.isTeleopEnabled()
                      && snapToSpeaker.get()
                      &&
                      // Shooter is ready
                      shooterSubsystem.inTolerance()
                      && shooterSubsystem.getSetpoint() == SetpointConstants.SHOOT_SHOOTER_VELOCITY
                      &&
                      // Wrist/elevator is ready
                      wristSubsystem.atGoal()
                      && elevatorSubsystem.atGoal()
                      &&
                      // Drive is locked
                      snapController.atSetpoint();
                },
                new SlidePattern(0.5, Color.kPurple, Color.kBlack)),
            // Purple if we are locked to apriltag with a note
            new LEDState(
                () ->
                    transportSubsystem.atSensor()
                        && photonSubsystem.getDistanceSpeaker().isPresent()
                        && DriverStation.isTeleopEnabled(),
                new SolidPattern(Color.kPurple)),
            // Green if we can go under the stage
            new LEDState(
                () ->
                    DriverStation.isTeleopEnabled()
                        && elevatorSubsystem.atBottom()
                        && wristSubsystem.atBottom(),
                new SolidPattern(Color.kGreen)),
            // Default disabled pattern
            new LEDState(
                DriverStation::isDisabled,
                new AlternatePattern(
                    8.0,
                    new SlidePattern(8.0 / 2.0, Color.kDarkRed, Color.kWhite),
                    new SlidePattern(8.0 / 2.0, Color.kWhite, Color.kDarkRed))));

    ledSubsystem.setDefaultCommand(
        new LEDStateMachineCommand(new SolidPattern(Color.kBlack), ledStates, ledSubsystem));
  }

  private final AtomicBoolean snapToSpeaker = new AtomicBoolean();

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
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                IntakingCommands.intakeUntilDetectedNoSlap(intakeSubsystem, transportSubsystem),
                ElevatorWristCommands.elevatorWristIntakePosition(
                    elevatorSubsystem, wristSubsystem)));
    driverController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                IntakingCommands.intakeUntilDetected(
                    intakeSubsystem, slapdownSuperstructure, transportSubsystem),
                ElevatorWristCommands.elevatorWristIntakePosition(
                    elevatorSubsystem, wristSubsystem)))
        .onFalse(slapdownSuperstructure.setUpCommand());
    driverController.circle().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());
    driverController
        .a()
        .whileTrue(
            Commands.parallel(
                ScoringCommands.reverseShooterTransportCommand(
                    shooterSubsystem, transportSubsystem),
                intakeSubsystem.setIntakeVoltageCommand(-6.0)));
    driverController
        .x()
        .onTrue(Commands.runOnce(() -> signalHumanPlayer.set(true)).withName("SignalHumanPlayer"))
        .onFalse(
            Commands.sequence(
                    Commands.waitSeconds(1.5), Commands.runOnce(() -> signalHumanPlayer.set(false)))
                .withName("SignalHumanPlayerOff"));
    driverController
        .b()
        .whileTrue(
            Commands.parallel(
                    Commands.run(() -> snapToSpeaker.set(true))
                        .finallyDo(() -> snapToSpeaker.set(false)),
                    ElevatorWristCommands.elevatorWristDynamicCommand(
                        () -> SetpointConstants.REGULAR_SHOT_ELEVATOR_HEIGHT_METERS,
                        () -> {
                          OptionalDouble distance = photonSubsystem.getDistanceSpeaker();
                          // TODO: Filter this so it doesn't start to go down if tag goes out for a
                          // small time
                          if (distance.isEmpty()) {
                            return SetpointConstants.CLOSE_SPEAKER_WRIST_ANGLE_RADIANS;
                          }
                          return SetpointConstants.REGULAR_SHOT_WRIST_SETPOINT_TABLE.get(
                              distance.getAsDouble());
                        },
                        elevatorSubsystem,
                        wristSubsystem))
                .withName("SnapToSpeakerAndWrist"));
    // driverController
    //     .y()
    //     .whileTrue(
    //         Commands.parallel(
    //                 Commands.run(() -> snapToSpeaker.set(true))
    //                     .finallyDo(() -> snapToSpeaker.set(false)),
    //                 ElevatorWristCommands.elevatorWristDynamicCommand(
    //                     () -> SetpointConstants.HIGH_SHOT_ELEVATOR_HEIGHT_METERS,
    //                     () -> {
    //                       OptionalDouble distance = photonSubsystem.getDistanceSpeaker();
    //                       // TODO: Filter this so it doesn't start to go down if tag goes out for
    // a
    //                       // small time
    //                       if (distance.isEmpty()) {
    //                         return SetpointConstants.CLOSE_SPEAKER_WRIST_ANGLE_RADIANS;
    //                       }
    //                       return SetpointConstants.HIGH_SHOT_WRIST_SETPOINT_TABLE.get(
    //                           distance.getAsDouble());
    //                     },
    //                     elevatorSubsystem,
    //                     wristSubsystem))
    // .withName("SnapToSpeakerAndWristHigh"));
  }

  private void configureOperatorBindings() {
    operatorController
        .square()
        .onTrue(
            Commands.parallel(
                    ScoringCommands.shootSetpointAmpCommand(shooterSubsystem),
                    Commands.sequence(
                        Commands.waitSeconds(0.1), transportSubsystem.setVoltageCommand(10)))
                .withName("ShootAmp"));
    operatorController
        .triangle()
        .onTrue(
            Commands.parallel(
                    ScoringCommands.shootSetpointShootingCommand(shooterSubsystem),
                    Commands.waitUntil(shooterSubsystem::inTolerance)
                        .andThen(MiscCommands.rumbleHIDCommand(operatorController.getHID())))
                .withName("ShootCloseSpeakerWithRumble"));
    operatorController.circle().onTrue(ScoringCommands.shootSetpointIdleCommand(shooterSubsystem));
    operatorController.x().onTrue(ScoringCommands.shootSetpointZeroCommand(shooterSubsystem));

    operatorController
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                IntakingCommands.intakeUntilDetectedNoSlap(intakeSubsystem, transportSubsystem),
                ElevatorWristCommands.elevatorWristIntakePosition(
                    elevatorSubsystem, wristSubsystem)));

    operatorController.options().onTrue(elevatorSubsystem.probeHomeCommand());
    operatorController
        .share()
        .onTrue(slapdownSuperstructure.getSlapdownRotationSubsystem().probeHomeCommand());

    operatorController
        .povRight()
        .onTrue(ElevatorWristCommands.elevatorWristExpelCommand(elevatorSubsystem, wristSubsystem));
    //    TunableDouble wristSetpoint = new TunableDouble("/wrist/setpoint", 45.0, true);
    //    operatorController.povUp().onTrue(ElevatorWristCommands.elevatorWristDynamicCommand(
    //        () -> SetpointConstants.REGULAR_SHOT_ELEVATOR_HEIGHT_METERS,
    //        () -> Units.degreesToRadians(wristSetpoint.get()) - WristConstants.WRIST_TO_SHOOTER,
    //        elevatorSubsystem,
    //        wristSubsystem));
    operatorController
        .povUp()
        .onTrue(
            ElevatorWristCommands.elevatorWristCloseSpeakerCommand(
                elevatorSubsystem, wristSubsystem));
    operatorController
        .povDown()
        .onTrue(ElevatorWristCommands.elevatorWristAmpCommand(elevatorSubsystem, wristSubsystem));
    operatorController
        .leftBumper()
        .onTrue(
            ElevatorWristCommands.elevatorWristClimbUpCommand(elevatorSubsystem, wristSubsystem));
    operatorController
        .rightBumper()
        .whileTrue(
            ElevatorWristCommands.elevatorWristClimbDownCommand(elevatorSubsystem, wristSubsystem));
    operatorController
        .leftTrigger()
        .onTrue(ElevatorWristCommands.elevatorWristZeroCommand(elevatorSubsystem, wristSubsystem));
  }

  private void configureDriving() {
    TunableDouble maxTranslationSpeedPercent =
        new TunableDouble("/speed/maxTranslation", 1.0, true);
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

    snapController.enableContinuousInput(-Math.PI, Math.PI);
    snapController.setTolerance(Units.degreesToRadians(1));

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
                () -> {
                  if (snapToSpeaker.get()) {
                    OptionalDouble result = photonSubsystem.getOffsetRadiansSpeaker();
                    OptionalDouble distance = photonSubsystem.getDistanceSpeaker();

                    if (result.isEmpty()) {
                      return 0;
                    } else {
                      double target = 0.0;
                      if (distance.isPresent()) {
                        target = Units.degreesToRadians(-2) * distance.getAsDouble();
                      }
                      return snapController.calculate(result.getAsDouble(), target);
                    }
                  }
                  return rotationLimiter.calculate(
                      RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
                          * maxAngularSpeedSupplier.getAsDouble());
                },
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
