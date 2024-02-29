package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.types.*;
import frc.robot.telemetry.wrappers.TelemetryCANCoder;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.*;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

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

  private static int instances = 0;

  private final int instanceId;

  private final DoubleTelemetryEntry driveVelocitySetpointEntry,
      steerPositionGoalEntry,
      feedForwardOutputEntry;
  private final BooleanTelemetryEntry activeSteerEntry, setToAbsoluteEntry, openLoopEntry;
  private final IntegerTelemetryEntry controlModeEntry;
  private final EventTelemetryEntry moduleEventEntry;

  private final Alert notSetToAbsoluteAlert,
      steerEncoderFaultAlert,
      steerMotorFaultAlert,
      driveMotorFaultAlert;

  private final TelemetryTalonFX driveMotor;
  private final TelemetryCANSparkMax steerMotor;
  private final TelemetryCANCoder absoluteSteerEncoder;

  private RelativeEncoder steerRelativeEncoder;
  private StatusSignal<Double> absoluteSteerPositionSignal;
  private StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
  private final double drivePositionConversion,
      driveVelocityConversion,
      steerPositionConversion,
      steerVelocityConversion;

  private final TunablePIDGains driveVelocityPIDGains;
  private final TunableFFGains driveVelocityFFGains;
  private final TunablePIDGains steerPositionPIDGains;
  private SimpleMotorFeedforward driveMotorFF;
  private SparkPIDController steerController;

  private boolean setToAbsolute = false;
  private double lastMoveTime = 0.0;
  private double lastAbsoluteResetTime = 0.0;

  /** Constructs a new Swerve Module using the given config */
  public SwerveModule(SwerveModuleConfiguration config, boolean tuningMode) {
    instanceId = instances++;

    // Initialize all telemetry entries
    String tableName = "/drive/modules/" + instanceId + "/";
    driveVelocitySetpointEntry =
        new DoubleTelemetryEntry(tableName + "driveVelocitySetpoint", true);
    openLoopEntry = new BooleanTelemetryEntry(tableName + "openLoop", tuningMode);
    feedForwardOutputEntry = new DoubleTelemetryEntry(tableName + "feedforwardOutput", tuningMode);
    steerPositionGoalEntry = new DoubleTelemetryEntry(tableName + "steerPositionGoal", true);
    activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", tuningMode);
    setToAbsoluteEntry = new BooleanTelemetryEntry(tableName + "setToAbsolute", true);
    controlModeEntry = new IntegerTelemetryEntry(tableName + "controlMode", false);
    moduleEventEntry = new EventTelemetryEntry(tableName + "events");

    // Initialize all alerts
    String alertPrefix = "Module " + instanceId + ": ";
    notSetToAbsoluteAlert =
        new Alert(alertPrefix + "Steer is not reset to absolute position", AlertType.ERROR);
    steerEncoderFaultAlert =
        new Alert(alertPrefix + "Steer encoder had a fault initializing", AlertType.ERROR);
    steerMotorFaultAlert =
        new Alert(alertPrefix + "Steer motor had a fault initializing", AlertType.ERROR);
    driveMotorFaultAlert =
        new Alert(alertPrefix + "Drive motor had a fault initializing", AlertType.ERROR);

    drivePositionConversion =
        (config.sharedConfiguration().wheelDiameterMeters() * Math.PI)
            / (config.sharedConfiguration().driveGearRatio());
    driveVelocityConversion = drivePositionConversion;
    steerPositionConversion = (Math.PI * 2) / (config.sharedConfiguration().steerGearRatio());
    steerVelocityConversion = steerPositionConversion / 60.0;

    driveVelocityPIDGains = config.sharedConfiguration().driveVelocityPIDGains();
    driveVelocityFFGains = config.sharedConfiguration().driveVelocityFFGains();
    steerPositionPIDGains = config.sharedConfiguration().steerPositionPIDGains();

    driveMotorFF = driveVelocityFFGains.createFeedforward();

    // Drive motor
    driveMotor =
        new TelemetryTalonFX(
            config.driveMotorID(),
            tableName + "driveMotor",
            config.sharedConfiguration().canBus(),
            tuningMode);
    configDriveMotor(config);

    // Register the drive motor with the orchestra
    OrchestraInstance.INSTANCE.addInstrument(driveMotor);

    // Steer encoder
    absoluteSteerEncoder =
        new TelemetryCANCoder(
            config.steerEncoderID(),
            tableName + "steerEncoder",
            config.sharedConfiguration().canBus(),
            tuningMode);
    configSteerEncoder(config);

    // Steer motor
    steerMotor =
        new TelemetryCANSparkMax(
            config.steerMotorID(),
            CANSparkMax.MotorType.kBrushless,
            tableName + "steerMotor",
            tuningMode);
    configSteerMotor(config);
    resetSteerToAbsolute(0.25);
  }

  private void configDriveMotor(SwerveModuleConfiguration config) {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        config.sharedConfiguration().driveCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.sharedConfiguration().driveVelocityPIDGains().setSlot(motorConfiguration.Slot0);
    motorConfiguration.MotorOutput.Inverted =
        config.driveMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Audio.AllowMusicDurDisable = true;

    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> driveMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          driveMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        () ->
            drivePositionSignal.setUpdateFrequency(
                config.sharedConfiguration().odometryFrequency()),
        () ->
            drivePositionSignal.getAppliedUpdateFrequency()
                == config.sharedConfiguration().odometryFrequency(),
        faultRecorder.run("Position signal update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        () ->
            driveVelocitySignal.setUpdateFrequency(
                config.sharedConfiguration().odometryFrequency()),
        () ->
            driveVelocitySignal.getAppliedUpdateFrequency()
                == config.sharedConfiguration().odometryFrequency(),
        faultRecorder.run("Velocity signal update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        driveMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        moduleEventEntry::append,
        "Drive motor module " + instanceId,
        faultRecorder.getFaultString());
    driveMotorFaultAlert.set(faultRecorder.hasFault());

    driveMotor.setLoggingPositionConversionFactor(drivePositionConversion);
    driveMotor.setLoggingVelocityConversionFactor(driveVelocityConversion);

    // Clear reset as this is on startup
    driveMotor.hasResetOccurred();
  }

  private void configSteerMotor(SwerveModuleConfiguration config) {
    steerRelativeEncoder = steerMotor.getEncoder();
    steerController = steerMotor.getPIDController();

    StringFaultRecorder faultRecorder = new StringFaultRecorder();

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        steerMotor::restoreFactoryDefaults,
        () -> true,
        faultRecorder.run("Factory default"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () ->
            steerMotor.setSmartCurrentLimit(
                config.sharedConfiguration().steerStallCurrentLimit(),
                config.sharedConfiguration().steerFreeCurrentLimit()),
        () -> true,
        faultRecorder.run("Current limit"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecord(
        () -> {
          steerMotor.setInverted(config.steerMotorInverted());
        },
        () -> steerMotor.getInverted() == config.steerMotorInverted(),
        faultRecorder.run("Invert"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerController.setP(steerPositionPIDGains.p.get()),
        () -> ConfigurationUtils.fpEqual(steerController.getP(), steerPositionPIDGains.p.get()),
        faultRecorder.run("P gain"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerController.setI(steerPositionPIDGains.i.get()),
        () -> ConfigurationUtils.fpEqual(steerController.getI(), steerPositionPIDGains.i.get()),
        faultRecorder.run("I gain"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerController.setD(steerPositionPIDGains.d.get()),
        () -> ConfigurationUtils.fpEqual(steerController.getD(), steerPositionPIDGains.d.get()),
        faultRecorder.run("D gain"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerController.setPositionPIDWrappingEnabled(true),
        () -> steerController.getPositionPIDWrappingEnabled(),
        faultRecorder.run("PID wrapping"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> steerController.setPositionPIDWrappingMinInput(-Math.PI),
        () ->
            ConfigurationUtils.fpEqual(steerController.getPositionPIDWrappingMinInput(), -Math.PI),
        faultRecorder.run("PID wrapping min input"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> steerController.setPositionPIDWrappingMaxInput(Math.PI),
        () -> ConfigurationUtils.fpEqual(steerController.getPositionPIDWrappingMaxInput(), Math.PI),
        faultRecorder.run("PID wrapping max input"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerRelativeEncoder.setPositionConversionFactor(steerPositionConversion),
        () ->
            ConfigurationUtils.fpEqual(
                steerRelativeEncoder.getPositionConversionFactor(), steerPositionConversion),
        faultRecorder.run("Position conversion factor"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerRelativeEncoder.setVelocityConversionFactor(steerVelocityConversion),
        () ->
            ConfigurationUtils.fpEqual(
                steerRelativeEncoder.getVelocityConversionFactor(), steerVelocityConversion),
        faultRecorder.run("Velocity conversion factor"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> steerMotor.setIdleMode(IdleMode.kBrake),
        () -> steerMotor.getIdleMode() == IdleMode.kBrake,
        faultRecorder.run("Idle mode"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () ->
            steerMotor.setPeriodicFramePeriod(
                PeriodicFrame.kStatus2, 1000 / SwerveConstants.ODOMETRY_FREQUENCY),
        () -> true,
        faultRecorder.run("Status 2 frame period"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        steerMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        moduleEventEntry::append,
        "Steer motor module " + instanceId,
        faultRecorder.getFaultString());
    steerMotorFaultAlert.set(faultRecorder.hasFault());
  }

  private void configSteerEncoder(SwerveModuleConfiguration config) {
    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();

    encoderConfiguration.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(config.steerOffsetRadians());
    encoderConfiguration.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    absoluteSteerPositionSignal = absoluteSteerEncoder.getAbsolutePosition();

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> absoluteSteerEncoder.getConfigurator().apply(encoderConfiguration),
        () -> {
          CANcoderConfiguration appliedConfig = new CANcoderConfiguration();
          absoluteSteerEncoder.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isCANcoderConfigurationEqual(encoderConfiguration, appliedConfig);
        },
        faultRecorder.run("Encoder configuration"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> ConfigurationUtils.explicitlySetSignalFrequency(absoluteSteerPositionSignal),
        () -> true,
        faultRecorder.run("Absolute signal update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        absoluteSteerEncoder::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        moduleEventEntry::append,
        "Steer encoder module " + instanceId,
        faultRecorder.getFaultString());
    steerEncoderFaultAlert.set(faultRecorder.hasFault());

    absoluteSteerEncoder.setLoggingPositionConversionFactor(Math.PI * 2);
    absoluteSteerEncoder.setLoggingVelocityConversionFactor(Math.PI * 2);
  }

  private void checkForSteerMotorReset() {
    // Steer motor lost power
    if (RobotBase.isReal() && steerMotor.getFault(CANSparkBase.FaultID.kHasReset)) {
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
    double absolutePosition =
        Units.rotationsToRadians(absoluteSteerPositionSignal.waitForUpdate(timeout).getValue());
    if (absoluteSteerPositionSignal.getStatus().isOK()) {
      REVLibError settingPositionError = steerRelativeEncoder.setPosition(absolutePosition);
      if (RaiderUtils.isRevOk(settingPositionError)) {
        setToAbsolute = true;
        lastAbsoluteResetTime = Timer.getFPGATimestamp();
        moduleEventEntry.append("Reset steer motor encoder to position: " + absolutePosition);
      } else {
        reportError("Failed to set SparkMax to absolute position: " + settingPositionError);
      }
    } else {
      reportError("Failed to get absolute position: " + absoluteSteerPositionSignal.getStatus());
    }
  }

  public boolean isSetToAbsolute() {
    return setToAbsolute;
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

  private double getDriveVelocityMetersPerSecond() {
    return driveVelocitySignal.refresh().getValue() * driveVelocityConversion;
  }

  private double getDriveMotorPositionMeters() {
    StatusSignal.refreshAll(drivePositionSignal, driveVelocitySignal);
    return StatusSignal.getLatencyCompensatedValue(drivePositionSignal, driveVelocitySignal)
        * drivePositionConversion;
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

  /**
   * @return The output voltage of the drive motor. Used for characterization
   */
  public double getActualDriveVoltage() {
    return driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue();
  }

  /**
   * Set the desired state for this swerve module
   *
   * @param state the desired state
   * @param activeSteer if steer should be active
   * @param openLoop if velocity control should be feed forward only. False if to use PIDF for
   *     velocity control.
   */
  public void setDesiredState(SwerveModuleState state, boolean activeSteer, boolean openLoop) {
    checkForDriveMotorReset();
    checkForSteerMotorReset();
    checkAndUpdateGains();

    controlModeEntry.append(SwerveModuleControlMode.NORMAL.logValue);

    if (state.speedMetersPerSecond != 0.0 || activeSteer) {
      lastMoveTime = Timer.getFPGATimestamp();
    }

    state = SwerveModuleState.optimize(state, getSteerAngle());

    setDriveReference(state.speedMetersPerSecond, openLoop);
    setSteerReference(state.angle.getRadians(), activeSteer);

    if (shouldResetToAbsolute()) {
      resetSteerToAbsolute();
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
        && Math.abs(Units.rotationsToRadians(absoluteSteerEncoder.getVelocity().getValue()))
            < Units.degreesToRadians(0.5);
  }

  public void setCharacterizationVoltage(double voltage) {
    controlModeEntry.append(SwerveModuleControlMode.CHARACTERIZATION.logValue);

    driveMotor.setControl(new VoltageOut(voltage));
    setSteerReference(0.0, true);
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
      steerController.setReference(targetAngleRadians, CANSparkMax.ControlType.kPosition);
    } else {
      steerMotor.setVoltage(0.0);
    }
  }

  private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
    driveVelocitySetpointEntry.append(targetVelocityMetersPerSecond);
    openLoopEntry.append(openLoop);

    double feedforwardValueVoltage = driveMotorFF.calculate(targetVelocityMetersPerSecond);
    feedForwardOutputEntry.append(feedforwardValueVoltage);

    if (openLoop) {
      driveMotor.setControl(new VoltageOut(feedforwardValueVoltage));
    } else {
      driveMotor.setControl(
          new VelocityVoltage(targetVelocityMetersPerSecond / driveVelocityConversion)
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

  /**
   * Reports error to event log and driver station. Prepends module details to DS report.
   *
   * @param message The message to report
   */
  private void reportError(String message) {
    DriverStation.reportError(message, false);
    moduleEventEntry.append(message);
  }

  /** Log all telemetry values. Should be called (only) in subsystem periodic */
  public void logValues() {
    driveMotor.logValues();
    steerMotor.logValues();
    absoluteSteerEncoder.logValues();

    notSetToAbsoluteAlert.set(!setToAbsolute);
    setToAbsoluteEntry.append(setToAbsolute);
  }
}
