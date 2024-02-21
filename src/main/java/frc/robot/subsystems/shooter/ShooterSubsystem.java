package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryPIDController;
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
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (voltage) -> setFlyVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private final TelemetryCANSparkFlex flywheelMotor =
      new TelemetryCANSparkFlex(
          SHOOTER_ID, CANSparkLowLevel.MotorType.kBrushless, "/shooter/motor", true);

  private RelativeEncoder flywheelEncoder;

  private final TunableTelemetryPIDController pidController =
      new TunableTelemetryPIDController("/shooter/pid", SHOOTER_PID_GAINS);
  private final SimpleMotorFeedforward feedforward = SHOOTER_FF_GAINS.createFeedforward();

  private final DoubleTelemetryEntry flyVoltageReq =
      new DoubleTelemetryEntry("/shooter/voltageReq", true);
  private final EventTelemetryEntry shooterEventEntry = new EventTelemetryEntry("/shooter/events");

  public ShooterSubsystem() {
    configMotor();

    setDefaultCommand(setVoltageCommand(0.0));
  }

  public void configMotor() {
    flywheelEncoder = flywheelMotor.getEncoder();
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

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage));
  }

  public Command runVelocityCommand(double setpointRotationsPerSecond) {
    double radiansPerSecond = Units.rotationsToRadians(setpointRotationsPerSecond);
    return this.run(
        () ->
            setVoltage(
                pidController.calculate(flywheelEncoder.getVelocity(), radiansPerSecond)
                    + feedforward.calculate(radiansPerSecond)));
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
    flywheelMotor.logValues();
  }
}
