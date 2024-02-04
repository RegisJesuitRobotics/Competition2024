package frc.robot.subsystems.transport;

import static frc.robot.Constants.TransportConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class TransportSubsystem extends SubsystemBase {

  private static Alert transportAlert;
  public final TelemetryCANSparkMax transportMotor =
      new TelemetryCANSparkMax(
          TRANSPORT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/motors/intake", TUNING_MODE);

  public void runShooterTransportVoltage(double voltage) {
    transportMotor.setVoltage(voltage);
  }

  private final DoubleTelemetryEntry topTransportVoltageReq =
      new DoubleTelemetryEntry("/shooterTransport/topVoltage", false);
  private final DoubleTelemetryEntry bottomTransportVoltageReq =
      new DoubleTelemetryEntry("/shooterTransport/bottomVoltage", false);

  public TransportSubsystem() {
    transportAlert = new Alert("Transport: ", Alert.AlertType.ERROR);
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

    transportAlert.set(faultInitializing);
  }

  public Command runTransportCommand() {
    return this.startEnd(
        () -> this.runShooterTransportVoltage(TRANSPORT_VOLTAGE),
        () -> this.runShooterTransportVoltage(0));
  }
}
