package frc.robot.subsystems.wrist;

import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class WristSubsystem extends SubsystemBase {
  private static final Alert wristAlert =
      new Alert("Wrist motor had a fault initializing", Alert.AlertType.ERROR);

  private final TelemetryCANSparkMax wristMotor =
      new TelemetryCANSparkMax(
          WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/wrist/motors", true);
  private final AbsoluteEncoder absoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final ArmFeedforward feedforward = WRIST_FF_GAINS.createFeedforward();
  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "/wrist/pid", WRIST_PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);

  private final DoubleTelemetryEntry wristVoltageReq =
      new DoubleTelemetryEntry("/wrist/voltage", MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry wristEventEntry = new EventTelemetryEntry("/wrist/events");

  public WristSubsystem() {
    configMotor();

    // Default command is safe state
    setDefaultCommand(setVotageCommand(0.0));
  }

  private void configMotor() {
    boolean faultInitializing =
        RaiderUtils.applyAndCheckRev(
            () -> wristMotor.setCANTimeout(250), () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            wristMotor::restoreFactoryDefaults, () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> wristMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> wristMotor.setIdleMode(IdleMode.kBrake),
            () -> wristMotor.getIdleMode() == IdleMode.kBrake,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> absoluteEncoder.setPositionConversionFactor(Math.PI * 2),
            () -> absoluteEncoder.getPositionConversionFactor() == Math.PI * 2,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> absoluteEncoder.setVelocityConversionFactor(Math.PI * 2),
            () -> absoluteEncoder.getVelocityConversionFactor() == Math.PI * 2,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    // Set the relative encoder too for logging
    double relativeEncoderConversionFactor = (2 * Math.PI) / WRIST_GEAR_RATIO;
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () ->
                wristMotor
                    .getEncoder()
                    .setPositionConversionFactor(relativeEncoderConversionFactor),
            () ->
                wristMotor.getEncoder().getPositionConversionFactor()
                    == relativeEncoderConversionFactor,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () ->
                wristMotor
                    .getEncoder()
                    .setVelocityConversionFactor(relativeEncoderConversionFactor / 60.0),
            () ->
                wristMotor.getEncoder().getVelocityConversionFactor()
                    == relativeEncoderConversionFactor / 60.0,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            wristMotor::burnFlashWithDelay,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    wristEventEntry.append("Wrist motor initialized" + (faultInitializing ? " with faults" : ""));
    wristAlert.set(faultInitializing);
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(absoluteEncoder.getPosition());
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public void stopMovement() {
    wristMotor.setVoltage(0);
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
    wristVoltageReq.append(voltage);
  }

  public Command setVotageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage));
  }

  public Command setPosiitonCommand(Rotation2d desiredPosition) {
    return this.run(
        () -> {
          controller.setGoal(desiredPosition.getRadians());
          double feedbackOutput = controller.calculate(getPosition().getRadians());
          TrapezoidProfile.State currentSetpoint = controller.getSetpoint();

          setVoltage(
              feedbackOutput
                  + feedforward.calculate(getPosition().getRadians(), currentSetpoint.velocity));
        });
  }

  @Override
  public void periodic() {
    wristMotor.logValues();
  }
}
