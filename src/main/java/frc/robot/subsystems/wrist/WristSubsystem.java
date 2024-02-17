package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class WristSubsystem extends SubsystemBase {
  private final Alert wristAlert;

  private final TelemetryCANSparkMax wristMotor =
      new TelemetryCANSparkMax(
          WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/wrist/motors", true);

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          WRIST_FF_GAINS.aFF.get(), WRIST_FF_GAINS.sFF.get(), WRIST_FF_GAINS.vFF.get());
  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "wrist/pid", WRIST_PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
  private final AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  public WristSubsystem() {
    wristAlert = new Alert("Wrist Motor Had Fault Initializing", Alert.AlertType.ERROR);
    configMotor();
  }

  private void configMotor() {

    boolean faultInitializing =
        RaiderUtils.applyAndCheckRev(
            () -> wristMotor.setCANTimeout(250),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            wristMotor::restoreFactoryDefaults,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> wristMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> wristMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
            () -> wristMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            wristMotor::burnFlashWithDelay,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    wristAlert.set(faultInitializing);

    wristEncoder.setPositionConversionFactor(Math.PI * 2);
    wristEncoder.setVelocityConversionFactor(Math.PI * 2 / 60);
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(wristEncoder.getPosition());
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

  @Override
  public void periodic() {
    double feedbackOutput = controller.calculate(getPosition().getRadians());

    TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
    double combinedOutput =
        feedbackOutput
            + feedforward.calculate(getPosition().getRadians(), currentSetpoint.velocity);

    wristMotor.setVoltage(combinedOutput);
  }
}
