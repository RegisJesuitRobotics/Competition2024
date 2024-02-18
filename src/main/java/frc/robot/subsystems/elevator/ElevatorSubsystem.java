package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class ElevatorSubsystem extends SubsystemBase {
  private static final Alert elevatorAlert =
      new Alert("Elevator motor had a fault initializing", Alert.AlertType.ERROR);

  private final TelemetryCANSparkMax elevatorMotor =
      new TelemetryCANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless, "/elevator/motor", true);

  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "elevator/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);

  private SimpleMotorFeedforward feedforward = FF_GAINS.createFeedforward();

  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final DigitalInput bottomLimit = new DigitalInput(ELEVATOR_LIMIT_SWITCH);

  private final BooleanTelemetryEntry bottomLimitEntry =
      new BooleanTelemetryEntry("/elevator/bottomLimit", true);
  private final EventTelemetryEntry elevatorEventEntry =
      new EventTelemetryEntry("/elevator/events");

  public ElevatorSubsystem() {
    configMotors();
  }

  private void configMotors() {
    boolean faultInitializing =
        RaiderUtils.applyAndCheckRev(
            () -> elevatorMotor.setCANTimeout(250),
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            elevatorMotor::restoreFactoryDefaults,
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheck(
            () -> elevatorMotor.setInverted(ELEVATOR_INVERTED),
            () -> elevatorMotor.getInverted() == ELEVATOR_INVERTED,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> elevatorMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> elevatorMotor.setIdleMode(IdleMode.kBrake),
            () -> true,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> elevatorEncoder.setPositionConversionFactor(METERS_PER_REV),
            () -> elevatorEncoder.getPositionConversionFactor() == METERS_PER_REV,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> elevatorEncoder.setVelocityConversionFactor(METERS_PER_REV / 60),
            () -> elevatorEncoder.getVelocityConversionFactor() == METERS_PER_REV / 60,
            MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            elevatorMotor::burnFlashWithDelay, () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);

    elevatorEventEntry.append(
        "Elevator motor initialized" + (faultInitializing ? " with faults" : ""));
    elevatorAlert.set(faultInitializing);
  }

  public void atBottomLimit() {
    if (bottomLimit.get()) {
      elevatorEncoder.setPosition(0);
    }
  }

  public void setDesiredPosition(double desiredPosition) {
    // TODO: CLAMP THIS
    controller.setGoal(desiredPosition);
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public void setEncoderPosition(double position) {
    elevatorEncoder.setPosition(position);
  }

  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public void stopMove() {
    elevatorMotor.setVoltage(0);
  }

  // TODO: Fix
  public Command runElevatorCommand(double voltage) {
    voltage = MathUtil.clamp(voltage, -0.5, 0.5);

    if (getPosition() > ELEVATOR_MIN_HEIGHT && getPosition() < ELEVATOR_MAX_HEIGHT) {
      double finalVoltage = voltage;
      return this.runEnd(() -> this.setVoltage(finalVoltage), () -> this.setVoltage(0));

    } else {
      return this.run(() -> this.setVoltage(0));
    }
  }

  // TODO: Fix
  public Command setElevatorPositionCommand(double position) {
    return this.run(
        () -> {
          controller.setGoal(position);
          double feedbackOutput = controller.calculate(getPosition());
          TrapezoidProfile.State currentSetpoint = controller.getSetpoint();

          setVoltage(
              feedbackOutput + feedforward.calculate(getPosition(), currentSetpoint.velocity));
        });
  }

  @Override
  public void periodic() {
    logValues();
  }

  private void logValues() {
    elevatorMotor.logValues();
    bottomLimitEntry.append(bottomLimit.get());
    if (FF_GAINS.hasChanged()) {
      feedforward = FF_GAINS.createFeedforward();
    }
  }
}
