package frc.robot.subsystems.slapdown;

import static frc.robot.Constants.SlapdownConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class SlapdownSubsystem extends SubsystemBase {

  private static final Alert rotationMotorAlert =
      new Alert("Slapdown rotation motor had a fault initializing", Alert.AlertType.ERROR);
  private static final Alert feederMotorAlert =
      new Alert("Slapdown feeder motor had a fault initializing", Alert.AlertType.ERROR);
  private final TelemetryCANSparkMax feederMotor =
      new TelemetryCANSparkMax(
          FEEDER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/slapdown/feeder/motor", false);
  private final TelemetryCANSparkFlex rotationMotor =
      new TelemetryCANSparkFlex(
          ROTATION_MOTOR_ID,
          CANSparkLowLevel.MotorType.kBrushless,
          "/slapdown/rotation/motor",
          true);

  private final SimpleMotorFeedforward rotationFF;
  private final TunableTelemetryProfiledPIDController rotationController =
      new TunableTelemetryProfiledPIDController(
          "/slapdown/rotation/controller", ROTATION_GAINS, ROTATION_TRAP_GAINS);
  private final RelativeEncoder rotationEncoder;

  private final EventTelemetryEntry slapdownEventEntry =
      new EventTelemetryEntry("/slapdown/events");

  public SlapdownSubsystem() {
    rotationEncoder = rotationMotor.getEncoder();
    rotationFF = ROTATION_FF_GAINS.createFeedforward();

    configMotors();
  }

  private void configMotors() {
    double rotationConversionFactor = (2 * Math.PI) / ROTATION_GEAR_RATIO;
    boolean rotationFaultInitializing =
        RaiderUtils.applyAndCheckRev(
            () -> rotationMotor.setCANTimeout(250),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    rotationFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            rotationMotor::restoreFactoryDefaults,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    rotationFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () ->
                rotationMotor.setSmartCurrentLimit(
                    ROTATION_STALL_MOTOR_CURRENT, ROTATION_FREE_MOTOR_CURRENT),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    rotationFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> rotationMotor.setIdleMode(IdleMode.kBrake),
            () -> rotationMotor.getIdleMode() == IdleMode.kBrake,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    rotationFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> rotationEncoder.setPositionConversionFactor(rotationConversionFactor),
            () -> rotationEncoder.getPositionConversionFactor() == rotationConversionFactor,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    rotationFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> rotationEncoder.setVelocityConversionFactor(rotationConversionFactor / 60),
            () -> rotationEncoder.getVelocityConversionFactor() == rotationConversionFactor / 60,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    rotationFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            rotationMotor::burnFlashWithDelay,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    slapdownEventEntry.append(
        "Slapdown rotation motor initialized" + (rotationFaultInitializing ? " with faults" : ""));
    rotationMotorAlert.set(rotationFaultInitializing);

    boolean feederFaultInitializing =
        RaiderUtils.applyAndCheckRev(
            () -> feederMotor.setCANTimeout(250),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    feederFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            feederMotor::restoreFactoryDefaults,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    feederFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () ->
                feederMotor.setSmartCurrentLimit(FEED_STALL_MOTOR_CURRENT, FEED_FREE_MOTOR_CURRENT),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    feederFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> feederMotor.setIdleMode(IdleMode.kCoast),
            () -> feederMotor.getIdleMode() == IdleMode.kCoast,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    feederFaultInitializing |=
        RaiderUtils.applyAndCheckRev(
            feederMotor::burnFlashWithDelay,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    slapdownEventEntry.append(
        "Slapdown feeder motor initialized" + (feederFaultInitializing ? " with faults" : ""));
    feederMotorAlert.set(feederFaultInitializing);
  }

  public void setFeederVoltage(double voltage) {
    feederMotor.setVoltage(voltage);
  }

  public void setRotationVoltage(double voltage) {
    rotationMotor.setVoltage(voltage);
  }

  private double getPosition() {
    return rotationEncoder.getPosition();
  }

  public Command setRotationGoalCommand(Rotation2d goal) {
    return this.run(
        () -> {
          rotationController.setGoal(goal.getRadians());
          double feedbackOutput = rotationController.calculate(getPosition());
          TrapezoidProfile.State currentSetpoint = rotationController.getSetpoint();

          setRotationVoltage(
              feedbackOutput + rotationFF.calculate(getPosition(), currentSetpoint.velocity));
        });
  }

  public Command setFeederVoltageCommand(double voltage) {
    return this.run(() -> this.setFeederVoltage(voltage));
  }

  @Override
  public void periodic() {
    rotationMotor.logValues();
    feederMotor.logValues();
  }
}
