package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

public class ShooterSubsystem extends SubsystemBase {
  // TODO I have no clue what the final cad looks like so these are all arbitrary

  private final TelemetryCANSparkMax motorFly =
      new TelemetryCANSparkMax(
          MOTOR_SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/shooter/motor", true);
  
  private final TelemetryCANSparkMax motorTransport =
      new TelemetryCANSparkMax(
          MOTOR_TRANSPORT_ID,
          CANSparkMaxLowLevel.MotorType.kBrushless,
          "/shooterTransport/motor",
          true);

  private final RelativeEncoder motorFlyEncoder = motorFly.getEncoder();

  private final SimpleMotorFeedforward FF =
      new SimpleMotorFeedforward(
          SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.sFF.get(), SHOOTER_FF_GAINS.vFF.get());

  private final DoubleTelemetryEntry motorFlyVoltageReq =
      new DoubleTelemetryEntry("/shooter/motorVoltage", false);

  private final DoubleTelemetryEntry motorTransportVoltageReq =
      new DoubleTelemetryEntry("/shooterTransport/motorVoltage", false);


  private final DigitalInput shooterFrisbeeSensor = new DigitalInput(SHOOTER_SENSOR);

  private double topVoltage = 0;

  public void setMotorFlyVoltage(double voltage) {
    motorFly.setVoltage(voltage);
  }


  public void setRPM(int rpm) {
    double forwardVol = FF.calculate(rpm);

    setMotorFlyVoltage(forwardVol);
  }

  public void runShooterTransportIn() {
    motorTransport.setVoltage(TRANSPORT_VOLTAGE);
  }

  public void shooterTransportStop() {
    motorTransport.setVoltage(0);
  }

  public void runShooterTransportOut() {
    motorTransport.setVoltage(-TRANSPORT_VOLTAGE);
  }

  public void TransportStop() {
    motorTransport.setVoltage(0);
  }

  public boolean isAtSensor() {
    if (shooterFrisbeeSensor.get()) {
      return true;
    } else {
      return false;
    }
  }

  public double getMotorEncoderVelocity() {
    return motorFlyEncoder.getVelocity();
  }

}
