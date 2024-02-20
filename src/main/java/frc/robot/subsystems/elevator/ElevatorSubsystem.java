package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

import java.sql.Time;

public class ElevatorSubsystem extends SubsystemBase {

  private final SysIdRoutine elevatorVoltageSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(4), null, null),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private static final Alert elevatorAlert =
      new Alert("Elevator motor had a fault initializing", Alert.AlertType.ERROR);

  private final TelemetryCANSparkMax elevatorMotor =
      new TelemetryCANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless, "/elevator/motor", true);

  private TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "elevator/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);

  private ElevatorFeedforward feedforward = FF_GAINS.createElevatorFeedforward();

  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final DigitalInput bottomLimit = new DigitalInput(ELEVATOR_LIMIT_SWITCH);

  private final DoubleTelemetryEntry voltageReq = new DoubleTelemetryEntry("/elevator/voltageReq", MiscConstants.TUNING_MODE);

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

  public boolean atGoal() {
    return controller.atGoal();
  }

  public void setEncoderPosition(double position) {
    elevatorEncoder.setPosition(position);
  }

  public void setVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
    voltageReq.append(voltage);
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public void stopMove() {
    elevatorMotor.setVoltage(0);
  }

  // TODO: Fix
  public Command setElevatorPositionCommand(double position) {
    return this.run(
        () -> {
          double feedbackOutput = controller.calculate(getPosition());
          TrapezoidProfile.State currentSetpoint = controller.getSetpoint();

          setVoltage(
              feedbackOutput + feedforward.calculate(currentSetpoint.velocity));
        }).beforeStarting(
            () -> {
              controller.reset(getPosition(), elevatorEncoder.getVelocity());
              controller.setGoal(position);
            }
    );
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return elevatorVoltageSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return elevatorVoltageSysId.dynamic(direction);
  }
  @Override
  public void periodic() {
    logValues();
  }

  private void logValues() {
    elevatorMotor.logValues();
    bottomLimitEntry.append(bottomLimit.get());

    if (FF_GAINS.hasChanged()) {
      feedforward = FF_GAINS.createElevatorFeedforward();
    }
    if (PID_GAINS.hasChanged() || TRAPEZOIDAL_PROFILE_GAINS.hasChanged()){
      controller = new TunableTelemetryProfiledPIDController(
              "elevator/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
    }
  }
}
