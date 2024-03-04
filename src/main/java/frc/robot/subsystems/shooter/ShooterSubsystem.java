package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class ShooterSubsystem extends SubsystemBase {
  private static final Alert flywheelMotorAlert =
      new Alert("Shooter motor had a fault initializing", AlertType.ERROR);

  private final SysIdRoutine shooterSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(10), Seconds.of(12), null),
          new SysIdRoutine.Mechanism(
              (voltage) -> setFlyVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private final SlewRateLimiter rateLimiter =
      new SlewRateLimiter(Units.rotationsPerMinuteToRadiansPerSecond(10000));

  private final TelemetryCANSparkFlex flywheelMotor =
      new TelemetryCANSparkFlex(
          SHOOTER_ID,
          CANSparkLowLevel.MotorType.kBrushless,
          "/shooter/motor",
          MiscConstants.TUNING_MODE);
  private final TelemetryCANSparkFlex flywheelMotorFollower =
      new TelemetryCANSparkFlex(
          SHOOTER_FOLLOWER_ID,
          CANSparkLowLevel.MotorType.kBrushless,
          "/shooter/follower",
          MiscConstants.TUNING_MODE);
  private RelativeEncoder flywheelEncoder;

  private final TunableTelemetryPIDController pidController =
      new TunableTelemetryPIDController("/shooter/pid", SHOOTER_PID_GAINS);
  private SimpleMotorFeedforward feedforward = SHOOTER_FF_GAINS.createFeedforward();

  private final DoubleTelemetryEntry flyVoltageReq =
      new DoubleTelemetryEntry("/shooter/voltageReq", true);
  private final BooleanTelemetryEntry atToleranceEntry =
      new BooleanTelemetryEntry("/shooter/inTolerance", Constants.MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry shooterEventEntry = new EventTelemetryEntry("/shooter/events");

  public ShooterSubsystem() {
    configMotor();

    setDefaultCommand(setVoltageCommand(0.0).ignoringDisable(true));
  }

  public void configMotor() {
    flywheelEncoder = flywheelMotor.getEncoder();
    StringFaultRecorder faultRecorderFollower = new StringFaultRecorder();
    double conversionFactor = Math.PI * 2 / SHOOTER_GEAR_RATIO;

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        flywheelMotor::restoreFactoryDefaults,
        () -> true,
        faultRecorder.run("Factory defaults"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limits"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> flywheelMotor.setInverted(INVERTED),
        () -> flywheelMotor.getInverted() == INVERTED,
        faultRecorder.run("Inverted"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
        () -> flywheelMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelEncoder.setPositionConversionFactor(conversionFactor),
        () ->
            ConfigurationUtils.fpEqual(
                flywheelEncoder.getPositionConversionFactor(), conversionFactor),
        faultRecorder.run("Position conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelEncoder.setVelocityConversionFactor(conversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                flywheelEncoder.getVelocityConversionFactor(), conversionFactor / 60),
        faultRecorder.run("Velocity conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        flywheelMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelMotorFollower.setCANTimeout(250),
        () -> true,
        faultRecorderFollower.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        flywheelMotorFollower::restoreFactoryDefaults,
        () -> true,
        faultRecorderFollower.run("Factory defaults"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelMotorFollower.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorderFollower.run("Current limits"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> flywheelMotorFollower.setInverted(INVERTED_FOLLOWER),
        () -> flywheelMotorFollower.getInverted() == INVERTED_FOLLOWER,
        faultRecorderFollower.run("Inverted"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> flywheelMotorFollower.setIdleMode(CANSparkMax.IdleMode.kCoast),
        () -> flywheelMotorFollower.getIdleMode() == CANSparkMax.IdleMode.kCoast,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        flywheelMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    flywheelMotorFollower.follow(flywheelMotor, INVERTED_FOLLOWER);
    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        shooterEventEntry::append,
        "Shooter motor",
        faultRecorder.getFaultString());
    flywheelMotorAlert.set(faultRecorder.hasFault());
  }

  public void setVoltage(double voltage) {
    flyVoltageReq.append(voltage);
    flywheelMotor.setVoltage(voltage);
  }

  public double getVelocity() {
    return flywheelEncoder.getVelocity();
  }

  public boolean inTolerance() {
    return Math.abs(getVelocity() - pidController.getSetpoint()) / (pidController.getSetpoint())
        < 0.05;
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage));
  }

  public Command runVelocityCommand(double setpointRadiansSecond) {
    return this.run(
            () -> {
              double rateLimited = rateLimiter.calculate(setpointRadiansSecond);
              setVoltage(
                  pidController.calculate(flywheelEncoder.getVelocity(), rateLimited)
                      + feedforward.calculate(rateLimited));
            })
        .beforeStarting(() -> rateLimiter.reset(flywheelEncoder.getVelocity()));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return shooterSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return shooterSysId.dynamic(direction);
  }

  public void setFlyVoltage(double voltage) {
    flywheelMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    if (SHOOTER_FF_GAINS.hasChanged()) {
      feedforward = SHOOTER_FF_GAINS.createFeedforward();
    }
    flywheelMotor.logValues();
    atToleranceEntry.append(inTolerance());
  }
}
