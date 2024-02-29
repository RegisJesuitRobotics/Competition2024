package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.types.*;
import frc.robot.telemetry.wrappers.TelemetryCANCoder;
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

  private final DoubleTelemetryEntry driveVelocitySetpointEntry, steerPositionGoalEntry;
  private final BooleanTelemetryEntry activeSteerEntry, openLoopEntry;
  private final IntegerTelemetryEntry controlModeEntry;
  private final EventTelemetryEntry moduleEventEntry;

  private final Alert steerEncoderFaultAlert, steerMotorFaultAlert, driveMotorFaultAlert;

  private final TelemetryTalonFX driveMotor;
  private final TelemetryTalonFX steerMotor;
  private final TelemetryCANCoder absoluteSteerEncoder;

  private StatusSignal<Double> absoluteSteerPositionSignal;
  private StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
  private StatusSignal<Double> steerPositionSignal, steerVelocitySignal;
  private final double driveConversion, steerConversion;

  private final double maxDriveVelocityMetersPerSecond;

  private final TunablePIDGains driveVelocityPIDGains;
  private final TunableFFGains driveVelocityFFGains;
  private final TunablePIDGains steerPositionPIDGains;
  private final TunableFFGains steerVelocityFFGains;

  /** Constructs a new Swerve Module using the given config */
  public SwerveModule(SwerveModuleConfiguration config, boolean tuningMode) {
    instanceId = instances++;

    // Initialize all telemetry entries
    String tableName = "/drive/modules/" + instanceId + "/";
    driveVelocitySetpointEntry =
        new DoubleTelemetryEntry(tableName + "driveVelocitySetpoint", true);
    openLoopEntry = new BooleanTelemetryEntry(tableName + "openLoop", tuningMode);
    steerPositionGoalEntry = new DoubleTelemetryEntry(tableName + "steerPositionGoal", true);
    activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", tuningMode);
    controlModeEntry = new IntegerTelemetryEntry(tableName + "controlMode", false);
    moduleEventEntry = new EventTelemetryEntry(tableName + "events");

    // Initialize all alerts
    String alertPrefix = "Module " + instanceId + ": ";
    steerEncoderFaultAlert =
        new Alert(alertPrefix + "Steer encoder had a fault initializing", AlertType.ERROR);
    steerMotorFaultAlert =
        new Alert(alertPrefix + "Steer motor had a fault initializing", AlertType.ERROR);
    driveMotorFaultAlert =
        new Alert(alertPrefix + "Drive motor had a fault initializing", AlertType.ERROR);

    driveConversion =
        (config.sharedConfiguration().wheelDiameterMeters() * Math.PI)
            / (config.sharedConfiguration().driveGearRatio());
    // Steer uses CANCoder which is one to one
    steerConversion = (Math.PI * 2);

    maxDriveVelocityMetersPerSecond = config.sharedConfiguration().freeSpeedMetersPerSecond();

    driveVelocityPIDGains = config.sharedConfiguration().driveVelocityPIDGains();
    driveVelocityFFGains = config.sharedConfiguration().driveVelocityFFGains();
    steerPositionPIDGains = config.sharedConfiguration().steerPositionPIDGains();
    steerVelocityFFGains = config.sharedConfiguration().steerVelocityFFGains();

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
        new TelemetryTalonFX(
            config.steerMotorID(),
            tableName + "steerMotor",
            config.sharedConfiguration().canBus(),
            tuningMode);
    configSteerMotor(config);
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

    driveMotor.setLoggingPositionConversionFactor(driveConversion);
    driveMotor.setLoggingVelocityConversionFactor(driveConversion);

    // Clear reset as this is on startup
    driveMotor.hasResetOccurred();
  }

  private void configSteerMotor(SwerveModuleConfiguration config) {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        config.sharedConfiguration().steerCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.Inverted =
        config.steerMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfiguration.Feedback.FeedbackRemoteSensorID = config.steerEncoderID();
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfiguration.Feedback.RotorToSensorRatio = config.sharedConfiguration().steerGearRatio();
    motorConfiguration.Feedback.SensorToMechanismRatio = config.sharedConfiguration().steerGearRatio();
    motorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

    config.sharedConfiguration().steerPositionPIDGains().setSlot(motorConfiguration.Slot0);
    config.sharedConfiguration().steerVelocityFFGains().setSlot(motorConfiguration.Slot0);
    // Came from the CTRE Swerve Module example
    motorConfiguration.MotionMagic.MotionMagicCruiseVelocity =
        100.0 / config.sharedConfiguration().steerGearRatio();
    motorConfiguration.MotionMagic.MotionMagicAcceleration =
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    motorConfiguration.MotionMagic.MotionMagicExpo_kV =
        0.12 * config.sharedConfiguration().steerGearRatio();
    motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;

    steerPositionSignal = steerMotor.getPosition();
    steerVelocitySignal = steerMotor.getVelocity();

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> steerMotor.getConfigurator().apply(motorConfiguration),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          steerMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfiguration, appliedConfig);
        },
        faultRecorder.run("Motor configuration"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        () ->
            steerPositionSignal.setUpdateFrequency(
                config.sharedConfiguration().odometryFrequency()),
        () ->
            steerPositionSignal.getAppliedUpdateFrequency()
                == config.sharedConfiguration().odometryFrequency(),
        faultRecorder.run("Position signal update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        () ->
            steerVelocitySignal.setUpdateFrequency(
                config.sharedConfiguration().odometryFrequency()),
        () ->
            steerVelocitySignal.getAppliedUpdateFrequency()
                == config.sharedConfiguration().odometryFrequency(),
        faultRecorder.run("Velocity signal update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        steerMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        moduleEventEntry::append,
        "Steer motor module " + instanceId,
        faultRecorder.getFaultString());
    steerMotorFaultAlert.set(faultRecorder.hasFault());

    steerMotor.setLoggingPositionConversionFactor(steerConversion);
    steerMotor.setLoggingVelocityConversionFactor(steerConversion);
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
        () -> absoluteSteerEncoder.getPosition().setUpdateFrequency(config.sharedConfiguration().odometryFrequency()),
        () -> absoluteSteerEncoder.getPosition().getAppliedUpdateFrequency() == config.sharedConfiguration().odometryFrequency(),
        faultRecorder.run("Position signal update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> absoluteSteerEncoder.getVelocity().setUpdateFrequency(config.sharedConfiguration().odometryFrequency()),
        () -> absoluteSteerEncoder.getVelocity().getAppliedUpdateFrequency() == config.sharedConfiguration().odometryFrequency(),
        faultRecorder.run("Velocity signal update frequency"),
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
    if (RobotBase.isReal() && steerMotor.hasResetOccurred()) {
      reportError("Steer motor reset occurred");
    }
  }

  private void checkForDriveMotorReset() {
    if (RobotBase.isReal() && driveMotor.hasResetOccurred()) {
      reportError("Drive motor reset occurred");
    }
  }

  /**
   * @return the rotation of the wheel
   */
  private Rotation2d getSteerAngle() {
    StatusSignal.refreshAll(steerPositionSignal, steerVelocitySignal);
    return Rotation2d.fromRadians(
        MathUtil.angleModulus(
            StatusSignal.getLatencyCompensatedValue(steerPositionSignal, steerVelocitySignal)
                * steerConversion));
  }

  private double getDriveVelocityMetersPerSecond() {
    return driveVelocitySignal.refresh().getValue() * driveConversion;
  }

  private double getDriveMotorPositionMeters() {
    StatusSignal.refreshAll(drivePositionSignal, driveVelocitySignal);
    return StatusSignal.getLatencyCompensatedValue(drivePositionSignal, driveVelocitySignal)
        * driveConversion;
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

    state = SwerveModuleState.optimize(state, getSteerAngle());

    // Account for steer error to reduce skew
    double steerErrorRadians = state.angle.getRadians() - getSteerAngle().getRadians();
    double cosineScalar = Math.cos(steerErrorRadians);
    if (cosineScalar < 0.0) {
      cosineScalar = 0.0;
    }
    state.speedMetersPerSecond *= cosineScalar;

    setDriveReference(state.speedMetersPerSecond, openLoop);
    setSteerReference(state.angle.getRadians(), activeSteer);
  }

  private final VoltageOut driveVoltageOut = new VoltageOut(0.0);

  public void setDriveCharacterizationVoltage(double voltage) {
    controlModeEntry.append(SwerveModuleControlMode.CHARACTERIZATION.logValue);

    driveMotor.setControl(driveVoltageOut.withOutput(voltage));
    setSteerReference(0.0, true);
  }

  private final VoltageOut steerVoltageOut = new VoltageOut(0.0);

  public void setRawVoltage(double driveVolts, double steerVolts) {
    controlModeEntry.append(SwerveModuleControlMode.RAW_VOLTAGE.logValue);

    driveMotor.setControl(driveVoltageOut.withOutput(driveVolts));
    steerMotor.setControl(steerVoltageOut.withOutput(steerVolts));
  }

  private final MotionMagicExpoVoltage steerMotionMagicExpoVoltage =
      new MotionMagicExpoVoltage(0.0).withUpdateFreqHz(0.0);

  private void setSteerReference(double targetAngleRadians, boolean activeSteer) {
    activeSteerEntry.append(activeSteer);
    steerPositionGoalEntry.append(targetAngleRadians);

    if (activeSteer) {
      steerMotor.setControl(
          steerMotionMagicExpoVoltage.withPosition(targetAngleRadians * steerConversion));
    } else {
      steerMotor.setVoltage(0.0);
    }
  }

  private final VelocityVoltage driveVelocityVoltage =
      new VelocityVoltage(0.0).withUpdateFreqHz(0.0);

  private void setDriveReference(double targetVelocityMetersPerSecond, boolean openLoop) {
    driveVelocitySetpointEntry.append(targetVelocityMetersPerSecond);
    openLoopEntry.append(openLoop);

    if (openLoop) {
      driveMotor.setControl(
          driveVoltageOut.withOutput(
              targetVelocityMetersPerSecond / maxDriveVelocityMetersPerSecond * 12.0));
    } else {
      driveMotor.setControl(
          driveVelocityVoltage.withVelocity(targetVelocityMetersPerSecond / driveConversion));
    }
  }

  private void checkAndUpdateGains() {
    if (driveVelocityPIDGains.hasChanged() || driveVelocityFFGains.hasChanged()) {
      Slot0Configs newSlotConfig = new Slot0Configs();
      driveVelocityPIDGains.setSlot(newSlotConfig);
      driveVelocityFFGains.setSlot(newSlotConfig);
      driveMotor.getConfigurator().apply(newSlotConfig);

      moduleEventEntry.append("Updated drive gains due to value change");
    }

    if (steerPositionPIDGains.hasChanged()) {
      Slot0Configs newSlotConfig = new Slot0Configs();
      steerPositionPIDGains.setSlot(newSlotConfig);
      steerVelocityFFGains.setSlot(newSlotConfig);
      steerMotor.getConfigurator().apply(newSlotConfig);

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
  }
}
