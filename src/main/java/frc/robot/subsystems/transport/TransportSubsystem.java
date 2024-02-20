package frc.robot.subsystems.transport;

import static frc.robot.Constants.TransportConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class TransportSubsystem extends SubsystemBase {

  private static final Alert transportAlert =
      new Alert("Transport motor had a fault initializing", Alert.AlertType.ERROR);
  private static final DigitalInput shooterSensor = new DigitalInput(SHOOTER_SENSOR_ID);
  public final TelemetryCANSparkMax transportMotor =
      new TelemetryCANSparkMax(
          TRANSPORT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/transport/motor", false);

  public void runShooterTransportVoltage(double voltage) {
    transportMotor.setVoltage(voltage);
  }

  private final BooleanTelemetryEntry sensorEntry = new BooleanTelemetryEntry("/transport/sensor", true);

  private final DoubleTelemetryEntry topTransportVoltageReq =
      new DoubleTelemetryEntry("/transport/voltageReq", Constants.MiscConstants.TUNING_MODE);

  private final EventTelemetryEntry transportEventEntry =
      new EventTelemetryEntry("/transport/events");

  public TransportSubsystem() {
    configMotor();
  }

  public void configMotor() {
    boolean faultInitializing = false;
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> transportMotor.setCANTimeout(250),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            transportMotor::restoreFactoryDefaults,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> transportMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> transportMotor.setIdleMode(CANSparkMax.IdleMode.kBrake),
            () -> transportMotor.getIdleMode() == CANSparkMax.IdleMode.kBrake,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            transportMotor::burnFlashWithDelay,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    transportEventEntry.append(
        "Transport motor initialized" + (faultInitializing ? " with faults" : ""));
    transportAlert.set(faultInitializing);
  }

  public boolean atSensor() {
    return shooterSensor.get();
  }

  public Command runTransportOutCommand() {
    return this.startEnd(
        () -> this.runShooterTransportVoltage(TRANSPORT_VOLTAGE),
        () -> this.runShooterTransportVoltage(0));
  }

  @Override
  public void periodic() {
    transportMotor.logValues();
    sensorEntry.append(atSensor());
  }
}
