package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

public class WristSubsystem extends SubsystemBase {

  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(WRIST_ENCODER_ID_A);

  private final DigitalInput wristSwitch = new DigitalInput(WRIST_SWITCH_ID);
  private final TelemetryCANSparkMax wristMotor =
      new TelemetryCANSparkMax(
          WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/wrist/motors", true);

<<<<<<< HEAD
  private final ArmFeedforward feedforward =
=======
  private final ArmFeedForward feedforward =
>>>>>>> 83696f14c15fa740f27dfef23cc8265f01b9937a
      new ArmFeedforward(
          WRIST_FF_GAINS.aFF.get(), WRIST_FF_GAINS.sFF.get(), WRIST_FF_GAINS.vFF.get());

  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "wrist/pid", WRIST_PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
          
  private final RelativeEncoder relativeEncoder = wristMotor.getEncoder();

  public WristSubsystem() {
    // TODO: FIGURE MAX AND MIN ROT
    absoluteEncoder.setDutyCycleRange(0, 0);
  }

  public boolean atTransportAngle() {
    return absoluteEncoder.get() == 0;
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(relativeEncoder.getPosition());
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public void stopMovement() {
    wristMotor.setVoltage(0);
  }

  public void setDesiredPosition(Rotation2d desiredPosition) {

    controller.setGoal(desiredPosition.getRadians());
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    double feedbackOutput = controller.calculate(getPosition().getRadians());

    TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
    double combinedOutput =
        feedbackOutput
            + feedforward.calculate(getPosition().getRadians(), currentSetpoint.velocity);

    setVoltage(combinedOutput);
    if (wristSwitch.get()) {
      absoluteEncoder.reset();
    }
  }
}
