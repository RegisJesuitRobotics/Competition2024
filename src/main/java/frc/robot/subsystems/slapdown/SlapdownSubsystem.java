package frc.robot.subsystems.slapdown;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.SlapdownConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class SlapdownSubsystem extends SubsystemBase {

  private static final Alert rotationMotorAlert =
      new Alert("Slapdown rotation motor had a fault initializing", Alert.AlertType.ERROR);
  private static final Alert feederMotorAlert =
      new Alert("Slapdown feeder motor had a fault initializing", Alert.AlertType.ERROR);

  private DoubleTelemetryEntry rotationEncoderEntry =
      new DoubleTelemetryEntry("/slapdown/encoders", true);
  private final TelemetryCANSparkMax feederMotor =
      new TelemetryCANSparkMax(
          FEEDER_MOTOR_ID,
          MotorType.kBrushless,
          "/slapdown/feeder/motor",
          Constants.MiscConstants.TUNING_MODE);
  private final TelemetryCANSparkMax rotationMotor =
      new TelemetryCANSparkMax(
          ROTATION_MOTOR_ID,
          MotorType.kBrushless,
          "/slapdown/rotation/motor",
          Constants.MiscConstants.TUNING_MODE);
  private final SysIdRoutine slapdownRotationSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              (voltage) -> setRotationVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

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
        slapdownEventEntry::append,
        "Slapdown rotation motor",
        rotationFaultRecorder.getFaultString());
    rotationMotorAlert.set(rotationFaultRecorder.hasFault());

    double feederConversionFactor = (2 * Math.PI) / FEEDER_GEAR_RATIO;
    StringFaultRecorder feederFaultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.setCANTimeout(250),
        () -> true,
        feederFaultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        feederMotor::restoreFactoryDefaults,
        () -> true,
        feederFaultRecorder.run("Factory default"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.setSmartCurrentLimit(FEED_STALL_MOTOR_CURRENT, FEED_FREE_MOTOR_CURRENT),
        () -> true,
        feederFaultRecorder.run("Current limit"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.setIdleMode(IdleMode.kCoast),
        () -> feederMotor.getIdleMode() == IdleMode.kCoast,
        feederFaultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> feederMotor.setInverted(FEEDER_INVERTED),
        () -> feederMotor.getInverted() == FEEDER_INVERTED,
        feederFaultRecorder.run("Invert"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.getEncoder().setPositionConversionFactor(feederConversionFactor),
        () ->
            ConfigurationUtils.fpEqual(
                feederMotor.getEncoder().getPositionConversionFactor(), feederConversionFactor),
        feederFaultRecorder.run("Position conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> feederMotor.getEncoder().setVelocityConversionFactor(feederConversionFactor / 60),
        () ->
            ConfigurationUtils.fpEqual(
                feederMotor.getEncoder().getVelocityConversionFactor(),
                feederConversionFactor / 60),
        feederFaultRecorder.run("Velocity conversion factor"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        feederMotor::burnFlashWithDelay,
        () -> true,
        feederFaultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        feederFaultRecorder.hasFault(),
        slapdownEventEntry::append,
        "Slapdown feeder motor",
        feederFaultRecorder.getFaultString());
    feederMotorAlert.set(feederFaultRecorder.hasFault());
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return slapdownRotationSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return slapdownRotationSysId.dynamic(direction);
  }

  public Command setFeederVoltageCommand(double voltage) {
    return this.run(() -> this.setFeederVoltage(voltage));
  }

  @Override
  public void periodic() {
    rotationMotor.logValues();
    feederMotor.logValues();

    rotationEncoderEntry.append(getPosition());
  }
}
