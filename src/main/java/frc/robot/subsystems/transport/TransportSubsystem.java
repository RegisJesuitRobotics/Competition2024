package frc.robot.subsystems.transport;

import static frc.robot.Constants.TransportConstants.*;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

public class TransportSubsystem extends SubsystemBase {
  public final TelemetryCANSparkMax transportMotor =
      new TelemetryCANSparkMax(
          TRANSPORT_MOTOR_ID,
          CANSparkMaxLowLevel.MotorType.kBrushless,
          "/motors/intake",
          TUNING_MODE);
}
