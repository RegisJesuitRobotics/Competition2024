package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.commands.drive.LockModulesCommand;
import frc.robot.commands.drive.teleop.SwerveDriveCommand;
import frc.robot.hid.CommandNintendoSwitchController;
import frc.robot.hid.CommandXboxPlaystationController;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.SendableTelemetryManager;
import frc.robot.telemetry.tunable.gains.TunableDouble;
import frc.robot.utils.*;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();

    private final CommandNintendoSwitchController driverController = new CommandNintendoSwitchController(0);
    private final CommandXboxPlaystationController operatorController = new CommandXboxPlaystationController(1);

    private final ListenableSendableChooser<Command> driveCommandChooser = new ListenableSendableChooser<>();

    public RobotContainer() {

        configureDriverBindings();
        configureOperatorBindings();
        configureAutos();

        SendableTelemetryManager.getInstance().addSendable("/commandScheduler", CommandScheduler.getInstance());
    }

    private void configureAutos() {}

    private void configureDriverBindings() {
        configureDriving();

        driverController
                .home()
                .onTrue(RaiderCommands.runOnceAllowDisable(driveSubsystem::zeroHeading)
                        .withName("ZeroHeading"));
        driverController.minus().whileTrue(new LockModulesCommand(driveSubsystem).repeatedly());
    }

    private void configureOperatorBindings() {}

    private void configureDriving() {
        TunableDouble maxTranslationSpeedPercent = new TunableDouble("/speed/maxTranslation", 0.9, true);
        TunableDouble maxMaxAngularSpeedPercent = new TunableDouble("/speed/maxAngular", 0.6, true);

        DoubleSupplier maxTranslationalSpeedSuppler = () -> maxTranslationSpeedPercent.get()
                * DriveTrainConstants.MAX_VELOCITY_METERS_SECOND
                * (driverController.leftBumper().getAsBoolean() ? 0.5 : 1);
        DoubleSupplier maxAngularSpeedSupplier =
                () -> maxMaxAngularSpeedPercent.get() * DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_SECOND;

        SupplierSlewRateLimiter rotationLimiter =
                new SupplierSlewRateLimiter(() -> TeleopConstants.ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED);
        VectorRateLimiter vectorRateLimiter =
                new VectorRateLimiter(() -> TeleopConstants.TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED);
        Runnable resetRateLimiters = () -> {
            ChassisSpeeds currentSpeeds = driveSubsystem.getCurrentChassisSpeeds();
            vectorRateLimiter.reset(
                    new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond));
            rotationLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
        };

        driveCommandChooser.setDefaultOption(
                "Hybrid (Default to Field Relative & absolute control but use robot centric when holding button)",
                new SwerveDriveCommand(
                                () -> vectorRateLimiter.calculate(new Translation2d(
                                                RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftY()),
                                                RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftX()))
                                        .times(maxTranslationalSpeedSuppler.getAsDouble())),
                                () -> rotationLimiter.calculate(
                                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
                                                * maxAngularSpeedSupplier.getAsDouble()),
                                driverController.rightBumper().negate(),
                                driveSubsystem)
                        .beforeStarting(resetRateLimiters));

        driveCommandChooser.addOption(
                "Robot Orientated",
                new SwerveDriveCommand(
                                () -> vectorRateLimiter.calculate(new Translation2d(
                                                RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftY()),
                                                RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getLeftX()))
                                        .times(maxTranslationalSpeedSuppler.getAsDouble())),
                                () -> rotationLimiter.calculate(
                                        RaiderMathUtils.deadZoneAndCubeJoystick(-driverController.getRightX())
                                                * maxAngularSpeedSupplier.getAsDouble()),
                                () -> false,
                                driveSubsystem)
                        .beforeStarting(resetRateLimiters));

        SendableTelemetryManager.getInstance().addSendable("/drive/DriveStyle", driveCommandChooser);

        evaluateDriveStyle(driveCommandChooser.getSelected());
        new Trigger(driveCommandChooser::hasNewValue)
                .onTrue(RaiderCommands.runOnceAllowDisable(() -> evaluateDriveStyle(driveCommandChooser.getSelected()))
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
        return Commands.none();
    }

    /**
     * Called from Robot.java when the alliance is detected to have changed.
     * @param newAlliance the new alliance
     */
    public void onAllianceChange(Alliance newAlliance) {}
}
