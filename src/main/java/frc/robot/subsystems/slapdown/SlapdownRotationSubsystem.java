package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.SlapdownConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.BooleanTelemetryEntry;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class SlapdownRotationSubsystem extends SubsystemBase {

  private static final Alert rotationMotorAlert =
      new Alert("Slapdown rotation motor had a fault initializing", Alert.AlertType.ERROR);
  private boolean isHoming = false;
  private final Debouncer debouncer = new Debouncer(0.5);

  private final TelemetryCANSparkMax rotationMotor =
      new TelemetryCANSparkMax(
          ROTATION_MOTOR_ID,
          MotorType.kBrushless,
          "/slapdown/rotation/motor",
          Constants.MiscConstants.TUNING_MODE);

  private final SysIdRoutine slapdownRotationSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(2), Seconds.of(5), null),
          new SysIdRoutine.Mechanism(
              (voltage) -> setRotationVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private final ArmFeedforward rotationFF = ROTATION_FF_GAINS.createArmFeedforward();
  private final BooleanTelemetryEntry atLimitEntry =
      new BooleanTelemetryEntry("/slapdown/rotation/atLimit", true);

  private final BooleanTelemetryEntry isHomedEntry =
      new BooleanTelemetryEntry("/slapdown/rotation/isHomed", true);
  private final DoubleTelemetryEntry voltageRequestEntry =
      new DoubleTelemetryEntry(
          "/slapdown/rotation/voltageReq", Constants.MiscConstants.TUNING_MODE);
  private final TunableTelemetryProfiledPIDController rotationController =
      new TunableTelemetryProfiledPIDController(
          "/slapdown/rotation/controller", ROTATION_GAINS, ROTATION_TRAP_GAINS);

  private final RelativeEncoder rotationEncoder;
  private final DigitalInput rotationLimitSwitch = new DigitalInput(ROTATION_LIMIT_SWITCH_ID);

  private final EventTelemetryEntry eventEntry =
      new EventTelemetryEntry("/slapdown/rotation/events");

  private boolean isHomed = false;

  public SlapdownRotationSubsystem() {
    rotationEncoder = rotationMotor.getEncoder();
    configMotors();

    setDefaultCommand(
        setVoltageCommand(0.0).ignoringDisable(true).withName("SlapdownRotationDefault"));
  }

  private void configMotors() {
    StringFaultRecorder rotationFaultRecorder = new StringFaultRecorder();
    double rotationConversionFactor = (2 * Math.PI) / ROTATION_GEAR_RATIO;
    ConfigurationUtils.applyCheckRecordRev(
        () -> rotationMotor.setCANTimeout(250),
        () -> true,
        rotationFaultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        rotationMotor::restoreFactoryDefaults,
        () -> true,
        rotationFaultRecorder.run("Factory defaults"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () ->
            rotationMotor.setSmartCurrentLimit(
                ROTATION_STALL_MOTOR_CURRENT, ROTATION_FREE_MOTOR_CURRENT),
        () -> true,
        rotationFaultRecorder.run("Current limits"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> rotationMotor.setIdleMode(IdleMode.kBrake),
        () -> rotationMotor.getIdleMode() == IdleMode.kBrake,
        rotationFaultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> rotationMotor.setInverted(ROTATION_INVERTED),
        () -> rotationMotor.getInverted() == ROTATION_INVERTED,
        rotationFaultRecorder.run("Invert"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> rotationEncoder.setPositionConversionFactor(rotationConversionFactor),
        () ->
            ConfigurationUtils.fpEqual(
                rotationEncoder.getPositionConversionFactor(), rotationConversionFactor),
        rotationFaultRecorder.run("Position conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> rotationEncoder.setVelocityConversionFactor(rotationConversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                rotationEncoder.getVelocityConversionFactor(), rotationConversionFactor / 60),
        rotationFaultRecorder.run("Velocity conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        rotationMotor::burnFlashWithDelay,
        () -> true,
        rotationFaultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        rotationFaultRecorder.hasFault(),
        eventEntry::append,
        "Slapdown rotation motor",
        rotationFaultRecorder.getFaultString());
    rotationMotorAlert.set(rotationFaultRecorder.hasFault());
  }

  public void setRotationVoltage(double voltage) {
    rotationMotor.setVoltage(voltage);
    voltageRequestEntry.append(voltage);
  }

  private double getPosition() {
    return rotationEncoder.getPosition();
  }

  private boolean atLimit() {
    return !rotationLimitSwitch.get();
  }

  public boolean isHomed() {
    return isHomed;
  }

  public Command setRotationGoalCommand(Rotation2d goal) {
    return this.run(
            () -> {
              double feedbackOutput = rotationController.calculate(getPosition());
              TrapezoidProfile.State currentSetpoint = rotationController.getSetpoint();

              setRotationVoltage(
                  feedbackOutput
                      + rotationFF.calculate(currentSetpoint.position, currentSetpoint.velocity));
            })
        .beforeStarting(
            () -> {
              rotationController.setGoal(goal.getRadians());
              rotationController.reset(getPosition(), rotationEncoder.getVelocity());
            })
        .onlyIf(this::isHomed);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setRotationVoltage(voltage));
  }

  public Command probeHomeCommand() {
    return setVoltageCommand(-0.5)
        .until(this::atLimit)
        .beforeStarting(() -> isHoming = true)
        .finallyDo(() -> isHoming = false);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return slapdownRotationSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return slapdownRotationSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    if ((atLimit() && isHoming) || debouncer.calculate(atLimit())) {
      rotationEncoder.setPosition(ROTATION_UP_ANGLE);
      isHomed = true;
    }
    isHomedEntry.append(isHomed);
    atLimitEntry.append(atLimit());

    rotationMotor.logValues();
  }
}
