package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class WristSubsystem extends SubsystemBase {
private static int instance = 0;

private final Alert wristAlert;
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(WRIST_ENCODER_ID_A);

  private final DigitalInput wristSwitch = new DigitalInput(WRIST_SWITCH_ID);
  private final TelemetryCANSparkMax wristMotor =
      new TelemetryCANSparkMax(
          WRIST_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless, "/wrist/motors", true);

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          WRIST_FF_GAINS.aFF.get(), WRIST_FF_GAINS.sFF.get(), WRIST_FF_GAINS.vFF.get());
  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "wrist/pid", WRIST_PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
  private final RelativeEncoder relativeEncoder = wristMotor.getEncoder();

  public WristSubsystem() {

      absoluteEncoder.setDutyCycleRange(0, 0);
      int instanceID = instance++;

      wristAlert = new Alert("Module " + instanceID + ": ", Alert.AlertType.ERROR);
      configMotor();
  }
   private void configMotor(){

 boolean faultInitializing = false;
  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> wristMotor.setCANTimeout(250), () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );

  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  wristMotor::restoreFactoryDefaults, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  () ->
                          wristMotor.setSmartCurrentLimit(
                                  STALL_MOTOR_CURRENT,
                                  FREE_MOTOR_CURRENT),
                                  () -> true,


                  Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );


  faultInitializing |=
          RaiderUtils.applyAndCheckRev(
                  () -> wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
                  () -> wristMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
                  Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );

  faultInitializing |= RaiderUtils.applyAndCheckRev(
          () -> wristMotor.setPeriodicFramePeriod(
                  CANSparkMaxLowLevel.PeriodicFrame.kStatus2, (int)(1000 / ODOMETRY_FREQUENCY)),
          () -> true,
          Constants.MiscConstants.CONFIGURATION_ATTEMPTS
          );
  faultInitializing |= RaiderUtils.applyAndCheckRev(
          wristMotor::burnFlashWithDelay, () -> true, Constants.MiscConstants.CONFIGURATION_ATTEMPTS
  );

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
