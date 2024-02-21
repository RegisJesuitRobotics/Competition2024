package frc.robot.subsystems.transport;

import static frc.robot.Constants.TransportConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class TransportSubsystem extends SubsystemBase {

  private static final Alert transportAlert =
      new Alert("Transport motor had a fault initializing", Alert.AlertType.ERROR);
  private static final DigitalInput shooterSensor = new DigitalInput(SHOOTER_SENSOR_ID);
  public final TelemetryCANSparkMax transportMotor =
      new TelemetryCANSparkMax(
          TRANSPORT_MOTOR_ID,
          CANSparkLowLevel.MotorType.kBrushless,
          "/transport/motor",
          MiscConstants.TUNING_MODE);

  public void runShooterTransportVoltage(double voltage) {
    voltageReq.append(voltage);
    transportMotor.setVoltage(voltage);
  }

  private final BooleanTelemetryEntry sensorEntry =
      new BooleanTelemetryEntry("/transport/sensor", true);

  private final DoubleTelemetryEntry voltageReq =
      new DoubleTelemetryEntry("/transport/voltageReq", Constants.MiscConstants.TUNING_MODE);

  private final EventTelemetryEntry transportEventEntry =
      new EventTelemetryEntry("/transport/events");

  public TransportSubsystem() {
    configMotor();
    setDefaultCommand(setVoltageCommand(0.0));
  }

  public void configMotor() {
    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordRev(
        () -> transportMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        transportMotor::restoreFactoryDefaults,
        () -> true,
        faultRecorder.run("Factory default"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> transportMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limit"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> transportMotor.setInverted(INVERTED),
        () -> transportMotor.getInverted() == INVERTED,
        faultRecorder.run("Invert"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> transportMotor.setIdleMode(CANSparkMax.IdleMode.kBrake),
        () -> transportMotor.getIdleMode() == CANSparkMax.IdleMode.kBrake,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        transportMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        transportEventEntry::append,
        "Transport motor",
        faultRecorder.getFaultString());
    transportAlert.set(faultRecorder.hasFault());
  }

  public boolean atSensor() {
    return !shooterSensor.get();
  }

  public Command runTransportOutCommand() {
    return this.startEnd(
        () -> this.runShooterTransportVoltage(TRANSPORT_VOLTAGE),
        () -> this.runShooterTransportVoltage(0));
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> this.runShooterTransportVoltage(voltage));
  }

  @Override
  public void periodic() {
    transportMotor.logValues();
    sensorEntry.append(atSensor());
  }
}
