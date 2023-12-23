package frc.robot.subsystems.swerve;

import static frc.robot.utils.RaiderUtils.checkRevError;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  private final RelativeEncoder steerRelativeEncoder;
  private StatusSignal<Double> absoluteSteerPositionSignal;
  private StatusSignal<Double> drivePositionSignal, driveVelocitySignal;
  private final double drivePositionConversion,
      driveVelocityConversion,
      steerPositionConversion,
      steerVelocityConversion;

  private final SparkMaxPIDController steerController;
  private final TunablePIDGains driveVelocityPIDGains;
  private final TunableFFGains driveVelocityFFGains;
  private final TunablePIDGains steerPositionPIDGains;
  private SimpleMotorFeedforward driveMotorFF;

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
            config.driveMotorPort(),
            tableName + "driveMotor",
            config.sharedConfiguration().canBus(),
            tuningMode);
    configDriveMotor(config);

    // Steer encoder
    absoluteSteerEncoder =
        new CANcoder(config.steerEncoderPort(), config.sharedConfiguration().canBus());
    configSteerEncoder(config);

    // Steer motor
    steerMotor =
        new TelemetryCANSparkMax(
            config.steerMotorPort(),
            CANSparkMax.MotorType.kBrushless,
            tableName + "steerMotor",
            tuningMode);
    steerRelativeEncoder = steerMotor.getEncoder();
    steerController = steerMotor.getPIDController();
    configSteerMotor(config);

    resetSteerToAbsolute(0.25);
  }

  private void configDriveMotor(SwerveModuleConfiguration config) {
    driveMotor.setLoggingPositionConversionFactor(drivePositionConversion);
    driveMotor.setLoggingVelocityConversionFactor(driveVelocityConversion);

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
        RaiderUtils.applyAndCheck(
            () -> driveMotor.getConfigurator().apply(motorConfiguration).isOK(),
            () -> {
              TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
              driveMotor.getConfigurator().refresh(appliedConfig);
              return appliedConfig.equals(motorConfiguration);
            },
            MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheck(
            () ->
                drivePositionSignal
                    .setUpdateFrequency(config.sharedConfiguration().odometryFrequency())
                    .isOK(),
            () ->
                drivePositionSignal.getAppliedUpdateFrequency()
                    == config.sharedConfiguration().odometryFrequency(),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheck(
            () ->
                driveVelocitySignal
                    .setUpdateFrequency(config.sharedConfiguration().odometryFrequency())
                    .isOK(),
            () ->
                driveVelocitySignal.getAppliedUpdateFrequency()
                    == config.sharedConfiguration().odometryFrequency(),
            MiscConstants.CONFIGURATION_ATTEMPTS);

    driveMotor.optimizeBusUtilization();

    // Clear reset as this is on startup
    driveMotor.hasResetOccurred();
    driveMotorFaultAlert.set(faultInitializing);
    moduleEventEntry.append("Drive motor initialized" + (faultInitializing ? " with faults" : ""));
  }

  private void configSteerMotor(SwerveModuleConfiguration config) {
    boolean faultInitializing = false;
    for (int i = 0; i < MiscConstants.CONFIGURATION_ATTEMPTS; i++) {
      faultInitializing |= checkRevError(steerMotor.setCANTimeout(250));

      faultInitializing |= checkRevError(steerMotor.restoreFactoryDefaults());

      faultInitializing |=
          checkRevError(
              steerMotor.setSmartCurrentLimit(
                  config.sharedConfiguration().steerStallCurrentLimit(),
                  config.sharedConfiguration().steerFreeCurrentLimit()));

      faultInitializing |=
          checkRevError(
              steerMotor.setClosedLoopRampRate(config.sharedConfiguration().steerClosedLoopRamp()));

      steerMotor.setInverted(config.steerMotorInverted());

      faultInitializing |= checkRevError(steerController.setP(steerPositionPIDGains.p.get()));
      faultInitializing |= checkRevError(steerController.setD(steerPositionPIDGains.d.get()));
      faultInitializing |= checkRevError(steerController.setI(steerPositionPIDGains.i.get()));

      faultInitializing |=
          checkRevError(steerRelativeEncoder.setPositionConversionFactor(steerPositionConversion));
      faultInitializing |=
          checkRevError(steerRelativeEncoder.setVelocityConversionFactor(steerVelocityConversion));

      faultInitializing |= checkRevError(steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake));

      faultInitializing |=
          checkRevError(
              steerMotor.setPeriodicFramePeriod(
                  PeriodicFrame.kStatus2, 1000 / SwerveConstants.ODOMETRY_FREQUENCY));

      faultInitializing |= checkRevError(steerMotor.burnFlashWithDelay());

      if (!faultInitializing) {
        break;
      }
    }

    steerMotorFaultAlert.set(faultInitializing);
    moduleEventEntry.append("Steer motor initialized" + (faultInitializing ? " with faults" : ""));
  }

  private void configSteerEncoder(SwerveModuleConfiguration config) {
    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
    encoderConfiguration.MagnetSensor.MagnetOffset =
        Units.radiansToRotations(config.offsetRadians());

    absoluteSteerPositionSignal = absoluteSteerEncoder.getAbsolutePosition();

    boolean faultInitializing = false;
    for (int i = 0; i < MiscConstants.CONFIGURATION_ATTEMPTS; i++) {
      faultInitializing |=
          !absoluteSteerEncoder.getConfigurator().apply(encoderConfiguration).isOK();

      if (!faultInitializing) {
        break;
      }
    }

    steerEncoderFaultAlert.set(faultInitializing);
    moduleEventEntry.append(
        "Steer encoder initialized" + (faultInitializing ? " with faults" : ""));
  }

  /**
   * Reports error to event log and driver station. Prepends module details to DS report.
   *
   * @param message The message to report
   */
  private void reportError(String message) {
    moduleEventEntry.append(message);
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
    double startTime = Timer.getFPGATimestamp();
    setToAbsolute = false;

    double absolutePosition;
    boolean gotAbsolutePosition = false;
    do {
      absolutePosition = absoluteSteerPositionSignal.waitForUpdate(timeout).getValue();
      if (absoluteSteerPositionSignal.getStatus().isOK()) {
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
    checkForSteerMotorReset();
    checkForDriveMotorReset();
    checkAndUpdateGains();

    controlModeEntry.append(SwerveModuleControlMode.NORMAL.logValue);

    double currentTime = Timer.getFPGATimestamp();

    if (state.speedMetersPerSecond != 0.0 || activeSteer) {
      lastMoveTime = currentTime;
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
      steerController.setReference(
          RaiderMathUtils.calculateContinuousInputSetpoint(
              getSteerAngleRadiansNoWrap(), targetAngleRadians),
          CANSparkMax.ControlType.kPosition);
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

  /** Log all telemetry values. Should be called (only) in subsystem periodic */
  public void logValues() {
    driveMotor.logValues();
    steerMotor.logValues();

    notSetToAbsoluteAlert.set(!setToAbsolute);
    absoluteHeadingEntry.append(absoluteSteerPositionSignal.refresh().getValue());
    setToAbsoluteEntry.append(setToAbsolute);
  }
}
