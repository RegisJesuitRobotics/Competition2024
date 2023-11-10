package frc.robot.subsystems.swerve;

import static frc.robot.utils.RaiderUtils.checkRevError;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.types.IntegerTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigTimeout;
import frc.robot.utils.RaiderMathUtils;
import frc.robot.utils.SwerveModuleConfiguration;

public class SwerveModule {
    private enum SwerveModuleControlMode {
        NORMAL(1),
        CHARACTERIZATION(2),
        RAW_VOLTAGE(3),
        DEAD_MODE(4);

        final int logValue;

        SwerveModuleControlMode(int logValue) {
            this.logValue = logValue;
        }
    }

    private static final int CAN_TIMEOUT_MS = 250;

    private static int instances = 0;

    private final int instanceId;
    private final DoubleTelemetryEntry driveVelocitySetpointEntry;
    private final BooleanTelemetryEntry openLoopEntry;
    private final DoubleTelemetryEntry steerPositionGoalEntry;
    private final DoubleTelemetryEntry feedForwardOutputEntry;
    private final BooleanTelemetryEntry activeSteerEntry;
    private final DoubleTelemetryEntry absoluteHeadingEntry;
    private final BooleanTelemetryEntry setToAbsoluteEntry;
    // 1 is regular, 2 is characterization, 3 is raw voltage, 4 is dead mode
    private final IntegerTelemetryEntry controlModeEntry;
    private final EventTelemetryEntry moduleEventEntry;

    private final Alert notSetToAbsoluteAlert;
    private final Alert steerEncoderFaultAlert;
    private final Alert steerMotorFaultAlert;
    private final Alert driveMotorFaultAlert;
    private final Alert inDeadModeAlert;

    private final TelemetryTalonFX driveMotor;
    private final TelemetryCANSparkMax steerMotor;
    private final CANcoder absoluteSteerEncoder;

    private final RelativeEncoder steerRelativeEncoder;
    private final StatusSignal<Double> absoluteEncoderSignal;

    private final double driveMotorConversionFactorPosition;
    private final double driveMotorConversionFactorVelocity;
    private final double steerMotorConversionFactorPosition;
    private final double steerMotorConversionFactorVelocity;
    private final double nominalVoltage;
    private final double openLoopMaxSpeed;
    private final double steerEncoderOffsetRadians;

    private final SparkMaxPIDController steerController;
    private final TunablePIDGains driveVelocityPIDGains;
    private final TunableFFGains driveVelocityFFGains;
    private final TunablePIDGains steerPositionPIDGains;
    private SimpleMotorFeedforward driveMotorFF;

    private boolean setToAbsolute = false;
    private boolean isDeadMode = false;
    private double lastMoveTime = 0.0;
    private double lastAbsoluteResetTime = 0.0;

    /** Constructs a new Swerve Module using the given config */
    public SwerveModule(SwerveModuleConfiguration config, boolean tuningMode) {
        instanceId = instances++;

        // Initialize all telemetry entries
        String tableName = "/drive/modules/" + instanceId + "/";
        driveVelocitySetpointEntry = new DoubleTelemetryEntry(tableName + "driveVelocitySetpoint", true);
        openLoopEntry = new BooleanTelemetryEntry(tableName + "openLoop", tuningMode);
        feedForwardOutputEntry = new DoubleTelemetryEntry(tableName + "feedforwardOutput", tuningMode);
        steerPositionGoalEntry = new DoubleTelemetryEntry(tableName + "steerPositionGoal", true);
        activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", tuningMode);
        absoluteHeadingEntry = new DoubleTelemetryEntry(tableName + "absoluteHeading", tuningMode);
        setToAbsoluteEntry = new BooleanTelemetryEntry(tableName + "setToAbsolute", true);
        controlModeEntry = new IntegerTelemetryEntry(tableName + "controlMode", false);
        moduleEventEntry = new EventTelemetryEntry(tableName + "events");

        String alertPrefix = "Module " + instanceId + ": ";
        notSetToAbsoluteAlert = new Alert(alertPrefix + "Steer is not reset to absolute position", AlertType.ERROR);
        steerEncoderFaultAlert = new Alert(alertPrefix + "Steer encoder had a fault initializing", AlertType.ERROR);
        steerMotorFaultAlert = new Alert(alertPrefix + "Steer motor had a fault initializing", AlertType.ERROR);
        driveMotorFaultAlert = new Alert(alertPrefix + "Drive motor had a fault initializing", AlertType.ERROR);
        inDeadModeAlert = new Alert(alertPrefix + "In dead mode", AlertType.WARNING);

        this.driveMotorConversionFactorPosition = (config.sharedConfiguration().wheelDiameterMeters() * Math.PI)
                / (config.sharedConfiguration().driveGearRatio());
        this.driveMotorConversionFactorVelocity = driveMotorConversionFactorPosition * 10.0;
        this.steerMotorConversionFactorPosition =
                (Math.PI * 2) / (config.sharedConfiguration().steerGearRatio());
        this.steerMotorConversionFactorVelocity = steerMotorConversionFactorPosition / 60.0;

        this.driveVelocityPIDGains = config.sharedConfiguration().driveVelocityPIDGains();
        this.driveVelocityFFGains = config.sharedConfiguration().driveVelocityFFGains();

        this.steerPositionPIDGains = config.sharedConfiguration().steerPositionPIDGains();

        // Drive motor

        this.driveMotor = new TelemetryTalonFX(
                config.driveMotorPort(),
                tableName + "driveMotor",
                config.sharedConfiguration().canBus(),
                tuningMode);
        configDriveMotor(config);

        // Steer encoder
        this.absoluteSteerEncoder = new CANcoder(
                config.steerEncoderPort(), config.sharedConfiguration().canBus());
        configSteerEncoder();
        this.absoluteEncoderSignal = absoluteSteerEncoder.getAbsolutePosition();

        // Steer motor
        this.steerMotor = new TelemetryCANSparkMax(
                config.steerMotorPort(), CANSparkMax.MotorType.kBrushless, tableName + "steerMotor", tuningMode);
        this.steerRelativeEncoder = steerMotor.getEncoder();
        this.steerController = steerMotor.getPIDController();
        configSteerMotor(config);

        this.nominalVoltage = config.sharedConfiguration().nominalVoltage();
        this.openLoopMaxSpeed = config.sharedConfiguration().openLoopMaxSpeed();
        this.steerEncoderOffsetRadians = config.offsetRadians();
        this.driveMotorFF = driveVelocityFFGains.createFeedforward();

        resetSteerToAbsolute(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
    }

    private void configDriveMotor(SwerveModuleConfiguration config) {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        // TODO
        boolean faultInitializing = false;
        do {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            motorConfiguration.CurrentLimits.SupplyCurrentLimit =
                    config.sharedConfiguration().driveContinuousCurrentLimit();
            motorConfiguration.CurrentLimits.SupplyCurrentThreshold =
                    config.sharedConfiguration().drivePeakCurrentLimit();
            motorConfiguration.CurrentLimits.SupplyTimeThreshold =
                    config.sharedConfiguration().drivePeakCurrentDurationSeconds();
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

            motorConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
                    config.sharedConfiguration().driveClosedLoopRamp();
            motorConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
                    config.sharedConfiguration().driveOpenLoopRamp();

            motorConfiguration.MotorOutput.Inverted = config.driveMotorInverted()
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

            faultInitializing |=
                    !driveMotor.getConfigurator().apply(motorConfiguration).isOK();

            Slot0Configs slot0Configs = new Slot0Configs();
            config.sharedConfiguration().driveVelocityPIDGains().setSlot(slot0Configs);
            faultInitializing |=
                    !driveMotor.getConfigurator().apply(slot0Configs).isOK();

            driveMotor.setLoggingPositionConversionFactor(driveMotorConversionFactorPosition);
            driveMotor.setLoggingVelocityConversionFactor(driveMotorConversionFactorVelocity);

            // Clear the reset of it starting up
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        driveMotor.hasResetOccurred();
        driveMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));
    }
    // TODO: same config??

    private void configSteerMotor(SwerveModuleConfiguration config) {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        boolean faultInitializing;

        do {
            faultInitializing = checkRevError(steerMotor.setCANTimeout(CAN_TIMEOUT_MS));

            faultInitializing |= checkRevError(steerMotor.restoreFactoryDefaults());

            faultInitializing |= checkRevError(steerMotor.setSmartCurrentLimit(
                    config.sharedConfiguration().steerStallCurrentLimit(),
                    config.sharedConfiguration().steerFreeCurrentLimit()));

            faultInitializing |= checkRevError(steerMotor.setClosedLoopRampRate(
                    config.sharedConfiguration().steerClosedLoopRamp()));

            steerMotor.setInverted(config.steerMotorInverted());

            faultInitializing |= checkRevError(steerController.setP(steerPositionPIDGains.p.get()));
            faultInitializing |= checkRevError(steerController.setD(steerPositionPIDGains.d.get()));
            faultInitializing |= checkRevError(steerController.setI(steerPositionPIDGains.i.get()));

            faultInitializing |=
                    checkRevError(steerRelativeEncoder.setPositionConversionFactor(steerMotorConversionFactorPosition));
            faultInitializing |=
                    checkRevError(steerRelativeEncoder.setVelocityConversionFactor(steerMotorConversionFactorVelocity));

            faultInitializing |= checkRevError(steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake));
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        // Clear the reset of it starting up

        steerMotorFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
    }

    private void configSteerEncoder() {
        ConfigTimeout configTimeout = new ConfigTimeout(MiscConstants.CONFIGURATION_TIMEOUT_SECONDS);
        CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
        boolean faultInitializing = false;
        do {
            faultInitializing |= !absoluteSteerEncoder
                    .getConfigurator()
                    .apply(encoderConfiguration)
                    .isOK();
        } while (faultInitializing && configTimeout.hasNotTimedOut());

        steerEncoderFaultAlert.set(faultInitializing);
        moduleEventEntry.append("Steer encoder initialized" + (faultInitializing ? " with faults" : ""));
    }

    /**
     * Reports error to event log and driver station. Prepends module details to DS report.
     *
     * @param message The message to report
     */
    private void reportError(String message) {
        moduleEventEntry.append(message);
        DriverStation.reportError(String.format("Module %d: %s", instanceId, message), false);
    }

    private void checkForSteerMotorReset() {
        // Steer motor lost power
        if (RobotBase.isReal() && absoluteSteerEncoder.hasResetOccurred()) {
            reportError("Steer motor reset occurred");
            setToAbsolute = false;
            resetSteerToAbsolute();
        }
    }

    private void checkForDriveMotorReset() {
        if (RobotBase.isReal() && driveMotor.hasResetOccurred()) {
            reportError("Drive motor reset occurred");
        }
    }

    /**
     * Resets the integrated encoder on the Steer motor to the absolute position of the CANCoder. Trys
     * only once, and if it fails, it will not try again until
     */
    public void resetSteerToAbsolute() {
        resetSteerToAbsolute(0.0);
    }

    /**
     * Resets the integrated encoder on the Steer motor to the absolute position of the CANCoder
     *
     * @param timeout The timeout in seconds to wait.
     */
    public void resetSteerToAbsolute(double timeout) {
        double startTime = Timer.getFPGATimestamp();
        setToAbsolute = false;

        double absolutePosition;
        boolean gotAbsolutePosition = false;
        do {
            // TODO: This is kinda dirty
            absolutePosition = getAbsoluteRadians(timeout);
            if (absoluteEncoderSignal.getError().isOK()) {
                gotAbsolutePosition = true;
            }
            if (gotAbsolutePosition) {

                REVLibError settingPositionError = steerRelativeEncoder.setPosition(absolutePosition);
                // If no error
                if (!checkRevError(settingPositionError) || timeout == 0.0) {
                    setToAbsolute = true;
                }
            }
        } while (!setToAbsolute && timeout != 0.0 && Timer.getFPGATimestamp() - startTime < timeout);

        if (!setToAbsolute) {
            reportError("CANCoder/TalonFX timed out while trying to get absolute position");
        } else {
            lastAbsoluteResetTime = Timer.getFPGATimestamp();
            moduleEventEntry.append("Reset steer motor encoder to position: " + absolutePosition);
        }

        notSetToAbsoluteAlert.set(!setToAbsolute);
    }

    public boolean isSetToAbsolute() {
        return setToAbsolute;
    }

    private double getAbsoluteRadians(double timeout) {
        if (timeout == 0.0) {
            absoluteEncoderSignal.refresh();
        } else {
            absoluteEncoderSignal.waitForUpdate(timeout);
        }

        return MathUtil.angleModulus(
                Units.rotationsToRadians(absoluteEncoderSignal.getValue()) + steerEncoderOffsetRadians);
    }

    private double getSteerAngleRadiansNoWrap() {
        return steerRelativeEncoder.getPosition();
    }

    /**
     * @return the rotation of the wheel
     */
    private Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(MathUtil.angleModulus(getSteerAngleRadiansNoWrap()));
    }

    private double getSteerVelocityRadiansPerSecond() {
        return steerRelativeEncoder.getVelocity();
    }

    private double getDriveVelocityMetersPerSecond() {
        return driveMotor.getVelocity().getValue() * driveMotorConversionFactorVelocity;
    }

    private double getDriveMotorPositionMeters() {
        return driveMotor.getPosition().getValue() * driveMotorConversionFactorPosition;
    }

    public void setDeadModule(boolean isDeadMode) {
        this.isDeadMode = isDeadMode;
        inDeadModeAlert.set(isDeadMode);

        steerMotor.setIdleMode(isDeadMode ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        // TODO
        //        driveMotor.setNeutralMode(isDeadMode ? NeutralMode.Coast : NeutralMode.Brake);
    }

    public void resetDriveMotorPosition() {
        //        driveMotor.setSelectedSensorPosition(0.0);
    }

    public CommandBase getToggleDeadModeCommand() {
        return Commands.runOnce(() -> setDeadModule(!isDeadMode))
                .ignoringDisable(true)
                .withName("ToggleDeadMode");
    }

    /**
     * @return the current state of the modules as reported from the encoders
     */
    public SwerveModuleState getActualState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getSteerAngle());
    }

    public SwerveModulePosition getActualPosition() {
        return new SwerveModulePosition(getDriveMotorPositionMeters(), getSteerAngle());
    }

    public double getActualDriveVoltage() {
        return driveMotor.getDutyCycle().getValue()
                * driveMotor.getSupplyVoltage().getValue();
    }

    /**
     * Set the desired state for this swerve module
     *
     *
     * @param state the desired state
     * @param activeSteer if steer should be active
     * @param openLoop if velocity control should be feed forward only. False if to use PIDF for
     *     velocity control.
     */
    public void setDesiredState(SwerveModuleState state, boolean activeSteer, boolean openLoop) {
        Robot.startWNode("SwerveModule[" + instanceId + "]#setDesiredState");
        Robot.startWNode("checkForResetAndGains");
        checkForSteerMotorReset();
        checkForDriveMotorReset();
        checkAndUpdateGains();
        Robot.endWNode();

        if (isDeadMode) {
            controlModeEntry.append(SwerveModuleControlMode.DEAD_MODE.logValue);
            return;
        }
        controlModeEntry.append(SwerveModuleControlMode.NORMAL.logValue);

        double currentTime = Timer.getFPGATimestamp();

        if (state.speedMetersPerSecond != 0.0 || activeSteer) {
            lastMoveTime = currentTime;
        }

        state = SwerveModuleState.optimize(state, getSteerAngle());

        Robot.startWNode("setDriveState");
        setDriveReference(state.speedMetersPerSecond, openLoop);
        Robot.endWNode();

        Robot.startWNode("setSteerState");
        setSteerReference(state.angle.getRadians(), activeSteer);
        Robot.endWNode();

        if (shouldResetToAbsolute()) {
            Robot.startWNode("resetSteerToAbsolute");
            resetSteerToAbsolute();
            Robot.endWNode();
        }
        Robot.endWNode();
    }

    public void setCharacterizationVoltage(double voltage) {
        controlModeEntry.append(SwerveModuleControlMode.CHARACTERIZATION.logValue);

        setSteerReference(0.0, true);

        driveMotor.setControl(new VoltageOut(voltage));
    }

    public void setRawVoltage(double driveVolts, double steerVolts) {
        controlModeEntry.append(SwerveModuleControlMode.RAW_VOLTAGE.logValue);

        driveMotor.setControl(new VoltageOut(driveVolts));
        steerMotor.setVoltage(steerVolts);
    }

    private void setSteerReference(double targetAngleRadians, boolean activeSteer) {
        activeSteerEntry.append(activeSteer);
        steerPositionGoalEntry.append(targetAngleRadians);

        if (activeSteer) {
            steerController.setReference(
                    RaiderMathUtils.calculateContinuousInputSetpoint(getSteerAngleRadiansNoWrap(), targetAngleRadians),
                    CANSparkMax.ControlType.kPosition);
        } else {
            steerMotor.setVoltage(0.0);
        }
    }

    private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
        driveVelocitySetpointEntry.append(targetVelocityMetersPerSecond);
        openLoopEntry.append(openLoop);

        if (openLoop) {
            driveMotor.setControl(new VoltageOut(targetVelocityMetersPerSecond / openLoopMaxSpeed * nominalVoltage));
        } else {
            double feedforwardValueVoltage = driveMotorFF.calculate(targetVelocityMetersPerSecond);
            feedForwardOutputEntry.append(feedforwardValueVoltage);
            driveMotor.setControl(
                    new PositionVoltage(targetVelocityMetersPerSecond / driveMotorConversionFactorVelocity)
                            .withFeedForward(feedforwardValueVoltage));
        }
    }

    private void checkAndUpdateGains() {
        if (driveVelocityPIDGains.hasChanged() || driveVelocityFFGains.hasChanged()) {
            Slot0Configs newSlotConfig = new Slot0Configs();
            driveVelocityPIDGains.setSlot(newSlotConfig);
            driveMotor.getConfigurator().apply(newSlotConfig);
            driveMotorFF = driveVelocityFFGains.createFeedforward();

            moduleEventEntry.append("Updated drive gains due to value change");
        }

        if (steerPositionPIDGains.hasChanged()) {
            steerController.setP(steerPositionPIDGains.p.get());
            steerController.setD(steerPositionPIDGains.d.get());
            steerController.setI(steerPositionPIDGains.i.get());

            moduleEventEntry.append("Updated steer gains due to value change");
        }
    }

    private boolean shouldResetToAbsolute() {
        double currentTime = Timer.getFPGATimestamp();
        // If we have not reset in 5 seconds, been still for 1.5 seconds and our steer
        // velocity is less than half a degree per second (could happen if we are being
        // pushed), reset to absolute
        return DriverStation.isDisabled()
                && currentTime - lastAbsoluteResetTime > 5.0
                && currentTime - lastMoveTime > 1.5
                && Math.abs(Units.rotationsToRadians(
                                absoluteSteerEncoder.getVelocity().getValue()))
                        < Units.degreesToRadians(0.5);
    }

    /** Log all telemetry values. Should be called (only) in subsystem periodic */
    public void logValues() {
        driveMotor.logValues();
        steerMotor.logValues();
        absoluteHeadingEntry.append(getAbsoluteRadians(0.0));
        setToAbsoluteEntry.append(setToAbsolute);
    }
}
