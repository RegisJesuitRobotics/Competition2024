package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.*;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class ElevatorSubsystem extends SubsystemBase {

  private final SysIdRoutine elevatorVoltageSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(4), null, null),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private static final Alert elevatorAlert =
      new Alert("Elevator main motor had a fault initializing", Alert.AlertType.ERROR);
  private static final Alert elevatorFollowerAlert = new Alert("Elevator follower motor had a fault initializing", Alert.AlertType.ERROR);

  private final TelemetryCANSparkMax elevatorMotor =
      new TelemetryCANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless, "/elevator/motor", true);
  private final TelemetryCANSparkMax elevatorMotorFollower = new TelemetryCANSparkMax(ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless, "/elevator/followerMotor", false);


  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "elevator/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);

  private ElevatorFeedforward feedforward = FF_GAINS.createElevatorFeedforward();

  private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
  private final DigitalInput bottomLimit = new DigitalInput(ELEVATOR_LIMIT_SWITCH);

  private final DoubleTelemetryEntry voltageReq =
      new DoubleTelemetryEntry("/elevator/voltageReq", MiscConstants.TUNING_MODE);

  private final BooleanTelemetryEntry isHomedEntry =
      new BooleanTelemetryEntry("/elevator/isHomed", true);
  private final BooleanTelemetryEntry bottomLimitEntry =
      new BooleanTelemetryEntry("/elevator/bottomLimit", MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry elevatorEventEntry =
      new EventTelemetryEntry("/elevator/main/events");
  private final EventTelemetryEntry elevatorFollowerEventEntry = new EventTelemetryEntry("elevator/follower/events");

  private boolean isHomed = false;

  public ElevatorSubsystem() {
    configMotors();
    controller.setTolerance(Units.inchesToMeters(0.5));
    setDefaultCommand(setVoltageCommand(0.0));
  }

  private void configMotors() {
    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    StringFaultRecorder followerFaultRecorder = new StringFaultRecorder();


    ConfigurationUtils.applyCheckRecordRev(
        () -> elevatorMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        elevatorMotor::restoreFactoryDefaults,
        () -> true,
        faultRecorder.run("Factory default"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> elevatorMotor.setInverted(MAIN_INVERTED),
        () -> elevatorMotor.getInverted() == MAIN_INVERTED,
        () -> elevatorEventEntry.append("Motor invert"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> elevatorMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limit"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> elevatorMotor.setIdleMode(IdleMode.kBrake),
        () -> true,
        faultRecorder.run("Idle mode"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> elevatorEncoder.setPositionConversionFactor(METERS_PER_REV),
        () ->
            ConfigurationUtils.fpEqual(
                elevatorEncoder.getPositionConversionFactor(), METERS_PER_REV),
        faultRecorder.run("Position conversion factor"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> elevatorEncoder.setVelocityConversionFactor(METERS_PER_REV / 60),
        () ->
            ConfigurationUtils.fpEqual(
                elevatorEncoder.getVelocityConversionFactor(), METERS_PER_REV / 60),
        faultRecorder.run("Velocity conversion factor"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        elevatorMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        elevatorEventEntry::append,
        "Elevator motor",
        faultRecorder.getFaultString());
    elevatorAlert.set(faultRecorder.hasFault());


    ConfigurationUtils.applyCheckRecordRev(
            () -> elevatorMotorFollower.setCANTimeout(250),
            () -> true,
            followerFaultRecorder.run("CAN timeout"),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
            elevatorMotorFollower::restoreFactoryDefaults,
            () -> true,
            followerFaultRecorder.run("Factory default"),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
            () -> elevatorMotorFollower.setInverted(FOLLOWER_INVERTED),
            () -> elevatorMotorFollower.getInverted() == FOLLOWER_INVERTED,
            () -> elevatorFollowerEventEntry.append("Motor invert"),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
            () -> elevatorMotorFollower.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            followerFaultRecorder.run("Current limit"),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
            () -> elevatorMotorFollower.setIdleMode(IdleMode.kBrake),
            () -> true,
            followerFaultRecorder.run("Idle mode"),
            MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
            elevatorMotorFollower::burnFlashWithDelay,
            () -> true,
            followerFaultRecorder.run("Burn flash"),
            MiscConstants.CONFIGURATION_ATTEMPTS);


    elevatorMotorFollower.follow(elevatorMotor, FOLLOWER_INVERTED);
    ConfigurationUtils.postDeviceConfig(
            followerFaultRecorder.hasFault(),
            elevatorEventEntry::append,
            "Elevator follower motor",
            faultRecorder.getFaultString());
    elevatorAlert.set(faultRecorder.hasFault());

  }

  public boolean atBottomLimit() {
    return !bottomLimit.get();
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

  public boolean isHomed() {
    return isHomed;
  }

  public Command setElevatorPositionCommand(double position) {
    double positionClamped = MathUtil.clamp(position, ELEVATOR_MIN_HEIGHT, ELEVATOR_MAX_HEIGHT);
    return RaiderCommands.ifCondition(this::isHomed)
        .then(
            this.run(
                    () -> {
                      double feedbackOutput = controller.calculate(getPosition());
                      TrapezoidProfile.State currentSetpoint = controller.getSetpoint();

                      setVoltage(feedbackOutput + feedforward.calculate(currentSetpoint.velocity));
                    })
                .beforeStarting(
                    () -> {
                      controller.reset(getPosition(), elevatorEncoder.getVelocity());
                      controller.setGoal(positionClamped);
                    }))
        .otherwise(Commands.none());
  }

  public Command probeHomeCommand() {
    return setVoltageCommand(-0.5).unless(this::isHomed).until(this::isHomed);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return elevatorVoltageSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return elevatorVoltageSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    // TODO: make this or elegant
    if (atBottomLimit()) {
      isHomed = true;
      setEncoderPosition(0.0);
    }
    logValues();
  }

  private void logValues() {
    elevatorMotor.logValues();
    bottomLimitEntry.append(atBottomLimit());
    isHomedEntry.append(isHomed);

    if (FF_GAINS.hasChanged()) {
      feedforward = FF_GAINS.createElevatorFeedforward();
    }
  }
}
