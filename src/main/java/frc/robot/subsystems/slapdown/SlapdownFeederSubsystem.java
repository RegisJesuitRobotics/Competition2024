package frc.robot.subsystems.slapdown;

import static frc.robot.Constants.SlapdownConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class SlapdownFeederSubsystem extends SubsystemBase {
  private static final Alert feederMotorAlert =
      new Alert("Slapdown feeder motor had a fault initializing", Alert.AlertType.ERROR);

  private final TelemetryCANSparkMax feederMotor =
      new TelemetryCANSparkMax(
          FEEDER_MOTOR_ID,
          MotorType.kBrushless,
          "/slapdown/feeder/motor",
          Constants.MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/slapdown/feeder/events");

  public SlapdownFeederSubsystem() {
    configMotors();

    setDefaultCommand(setVoltageCommand(0.0));
  }

  private void configMotors() {
    double feederConversionFactor = (2 * Math.PI) / FEEDER_GEAR_RATIO;
    StringFaultRecorder feederFaultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.setCANTimeout(250),
        () -> true,
        feederFaultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        feederMotor::restoreFactoryDefaults,
        () -> true,
        feederFaultRecorder.run("Factory default"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.setSmartCurrentLimit(FEED_STALL_MOTOR_CURRENT, FEED_FREE_MOTOR_CURRENT),
        () -> true,
        feederFaultRecorder.run("Current limit"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.setIdleMode(IdleMode.kCoast),
        () -> feederMotor.getIdleMode() == IdleMode.kCoast,
        feederFaultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> feederMotor.setInverted(FEEDER_INVERTED),
        () -> feederMotor.getInverted() == FEEDER_INVERTED,
        feederFaultRecorder.run("Invert"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.getEncoder().setPositionConversionFactor(feederConversionFactor),
        () ->
            ConfigurationUtils.fpEqual(
                feederMotor.getEncoder().getPositionConversionFactor(), feederConversionFactor),
        feederFaultRecorder.run("Position conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.getEncoder().setVelocityConversionFactor(feederConversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                feederMotor.getEncoder().getVelocityConversionFactor(),
                feederConversionFactor / 60),
        feederFaultRecorder.run("Velocity conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        feederMotor::burnFlashWithDelay,
        () -> true,
        feederFaultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        feederFaultRecorder.hasFault(),
        eventEntry::append,
        "Slapdown feeder motor",
        feederFaultRecorder.getFaultString());
    feederMotorAlert.set(feederFaultRecorder.hasFault());
  }

  public void setFeederVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setFeederVoltage(voltage));
  }

  @Override
  public void periodic() {
    feederMotor.logValues();
  }
}
