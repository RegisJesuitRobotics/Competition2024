package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
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
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.types.*;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.*;
import frc.robot.utils.Alert.AlertType;

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

  private final DoubleTelemetryEntry driveVelocitySetpointEntry,
      steerPositionGoalEntry,
      feedForwardOutputEntry,
      absoluteHeadingEntry;
  private final BooleanTelemetryEntry activeSteerEntry, setToAbsoluteEntry, openLoopEntry;
  private final IntegerTelemetryEntry controlModeEntry;
  private final EventTelemetryEntry moduleEventEntry;

  private final Alert notSetToAbsoluteAlert,
      steerEncoderFaultAlert,
      steerMotorFaultAlert,
      driveMotorFaultAlert;

  private final TelemetryTalonFX driveMotor;
  private final TelemetryCANSparkMax steerMotor;
  private final CANcoder absoluteSteerEncoder;

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
  private SparkMaxPIDController steerController;

  private boolean setToAbsolute = false;
  private double lastMoveTime = 0.0;
  private double lastAbsoluteResetTime = 0.0;

  /** Constructs a new Swerve Module using the given config */
  public SwerveModule(SwerveModuleConfiguration config, boolean tuningMode) {
    int instanceId = instances++;

    // Initialize all telemetry entries
    String tableName = "/drive/modules/" + instanceId + "/";
    driveVelocitySetpointEntry =
        new DoubleTelemetryEntry(tableName + "driveVelocitySetpoint", true);
    openLoopEntry = new BooleanTelemetryEntry(tableName + "openLoop", tuningMode);
    feedForwardOutputEntry = new DoubleTelemetryEntry(tableName + "feedforwardOutput", tuningMode);
    steerPositionGoalEntry = new DoubleTelemetryEntry(tableName + "steerPositionGoal", true);
    activeSteerEntry = new BooleanTelemetryEntry(tableName + "activeSteer", tuningMode);
    absoluteHeadingEntry = new DoubleTelemetryEntry(tableName + "absoluteHeading", tuningMode);
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
        new CANcoder(config.steerEncoderID(), config.sharedConfiguration().canBus());
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
        config.sharedConfiguration().driveContinuousCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentThreshold =
        config.sharedConfiguration().drivePeakCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyTimeThreshold =
        config.sharedConfiguration().drivePeakCurrentDurationSeconds();
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.sharedConfiguration().driveVelocityPIDGains().setSlot(motorConfiguration.Slot0);
    motorConfiguration.MotorOutput.Inverted =
        config.driveMotorInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();

    boolean faultInitializing = false;
    faultInitializing |=
        RaiderUtils.applyAndCheckCTRE(
            () -> driveMotor.getConfigurator().apply(motorConfiguration),
            () -> {
              TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
              driveMotor.getConfigurator().refresh(appliedConfig);
              return appliedConfig.equals(motorConfiguration);
            },
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckCTRE(
            () ->
                drivePositionSignal.setUpdateFrequency(
                    config.sharedConfiguration().odometryFrequency()),
            () ->
                drivePositionSignal.getAppliedUpdateFrequency()
                    == config.sharedConfiguration().odometryFrequency(),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckCTRE(
            () ->
                driveVelocitySignal.setUpdateFrequency(
                    config.sharedConfiguration().odometryFrequency()),
            () ->
                driveVelocitySignal.getAppliedUpdateFrequency()
                    == config.sharedConfiguration().odometryFrequency(),
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckCTRE(
            driveMotor::optimizeBusUtilization, () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);

    // Clear reset as this is on startup
    driveMotor.hasResetOccurred();
    driveMotorFaultAlert.set(faultInitializing);
    moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));

    driveMotor.setLoggingPositionConversionFactor(drivePositionConversion);
    driveMotor.setLoggingVelocityConversionFactor(driveVelocityConversion);
  }

  private void configSteerMotor(SwerveModuleConfiguration config) {
    steerRelativeEncoder = steerMotor.getEncoder();
    steerController = steerMotor.getPIDController();

    boolean faultInitializing = false;
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerMotor.setCANTimeout(250), () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            steerMotor::restoreFactoryDefaults, () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () ->
                steerMotor.setSmartCurrentLimit(
                    config.sharedConfiguration().steerStallCurrentLimit(),
                    config.sharedConfiguration().steerFreeCurrentLimit()),
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheck(
            () -> {
              steerMotor.setInverted(config.steerMotorInverted());
              return true;
            },
            () -> steerMotor.getInverted() == config.steerMotorInverted(),
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerController.setP(steerPositionPIDGains.p.get()),
            () -> steerController.getP() == steerPositionPIDGains.p.get(),
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerController.setI(steerPositionPIDGains.i.get()),
            () -> steerController.getI() == steerPositionPIDGains.i.get(),
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerController.setD(steerPositionPIDGains.d.get()),
            () -> steerController.getD() == steerPositionPIDGains.d.get(),
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerController.setPositionPIDWrappingEnabled(true),
            () -> steerController.getPositionPIDWrappingEnabled(),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerController.setPositionPIDWrappingMinInput(-Math.PI),
            () -> steerController.getPositionPIDWrappingMinInput() == -Math.PI,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerController.setPositionPIDWrappingMaxInput(Math.PI),
            () -> steerController.getPositionPIDWrappingMaxInput() == Math.PI,
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerRelativeEncoder.setPositionConversionFactor(steerPositionConversion),
            () -> steerRelativeEncoder.getPositionConversionFactor() == steerPositionConversion,
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerRelativeEncoder.setVelocityConversionFactor(steerVelocityConversion),
            () -> steerRelativeEncoder.getVelocityConversionFactor() == steerVelocityConversion,
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake),
            () -> steerMotor.getIdleMode() == CANSparkMax.IdleMode.kBrake,
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () ->
                steerMotor.setPeriodicFramePeriod(
                    PeriodicFrame.kStatus2, 1000 / SwerveConstants.ODOMETRY_FREQUENCY),
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            steerMotor::burnFlashWithDelay, () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);

    steerMotorFaultAlert.set(faultInitializing);
    moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
  }

  private void configSteerEncoder(SwerveModuleConfiguration config) {
    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
    encoderConfiguration.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(config.steerOffsetRadians());

    absoluteSteerPositionSignal = absoluteSteerEncoder.getAbsolutePosition();

    boolean faultInitializing =
        RaiderUtils.applyAndCheckCTRE(
            () -> absoluteSteerEncoder.getConfigurator().apply(encoderConfiguration),
            () -> {
              CANcoderConfiguration appliedConfig = new CANcoderConfiguration();
              absoluteSteerEncoder.getConfigurator().refresh(appliedConfig);
              return appliedConfig.equals(encoderConfiguration);
            },
            MiscConstants.CONFIGURATION_ATTEMPTS);

    steerEncoderFaultAlert.set(faultInitializing);
    moduleEventEntry.append(
        "Steer encoder initialized" + (faultInitializing ? " with faults" : ""));
  }

  private void checkForSteerMotorReset() {
    // Steer motor lost power
    if (RobotBase.isReal() && steerMotor.getFault(FaultID.kHasReset)) {
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
    double absolutePosition = absoluteSteerPositionSignal.waitForUpdate(timeout).getValue();
    if (absoluteSteerPositionSignal.getStatus().isOK()) {
      REVLibError settingPositionError = steerRelativeEncoder.setPosition(absolutePosition);
      if (!RaiderUtils.checkRevError(settingPositionError)) {
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

    notSetToAbsoluteAlert.set(!setToAbsolute);
    absoluteHeadingEntry.append(absoluteSteerPositionSignal.refresh().getValue());
    setToAbsoluteEntry.append(setToAbsolute);
  }

  public TalonFXSimState getDriveSimState() {
    return driveMotor.getSimState();
  }
}
