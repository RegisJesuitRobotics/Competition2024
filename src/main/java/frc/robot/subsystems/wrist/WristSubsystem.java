package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.WristConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class WristSubsystem extends SubsystemBase {
  private static final Alert wristAlert =
      new Alert("Wrist motor had a fault initializing", Alert.AlertType.ERROR);
  private final SysIdRoutine wristSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(Volts.per(Second).of(.5), Volts.of(2), null, null),
          new SysIdRoutine.Mechanism(
              (voltage) -> setVoltage(voltage.in(Volts)),
              null, // No log consumer, since data is recorded by URCL
              this));

  private DoubleTelemetryEntry absoluteEncoderEntry =
      new DoubleTelemetryEntry("/wrist/absoluteEncoder", true);

  private final TelemetryCANSparkFlex wristMotor =
      new TelemetryCANSparkFlex(
          WRIST_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/wrist/motors", true);
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(WRIST_ENCODER_PORT);

  private final ArmFeedforward feedforward = WRIST_FF_GAINS.createArmFeedforward();
  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "/wrist/pid", WRIST_PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);

  private final DoubleTelemetryEntry wristVoltageReq =
      new DoubleTelemetryEntry("/wrist/voltage", MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry wristEventEntry = new EventTelemetryEntry("/wrist/events");

  public WristSubsystem() {
    configMotor();
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    controller.setTolerance(Units.degreesToRadians(6));

    wristMotor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition());
    // Default command is safe state
    setDefaultCommand(setVoltageCommand(0.0));
  }

  private void configMotor() {
    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordRev(
        () -> wristMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        wristMotor::restoreFactoryDefaults,
        () -> true,
        faultRecorder.run("Factory default"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> wristMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limit"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> wristMotor.setInverted(INVERTED),
        () -> wristMotor.getInverted() == INVERTED,
        () -> wristEventEntry.append("Invert"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> wristMotor.setIdleMode(IdleMode.kBrake),
        () -> wristMotor.getIdleMode() == IdleMode.kBrake,
        faultRecorder.run("Idle mode"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    // Set the relative encoder too for logging
    double relativeEncoderConversionFactor = (2 * Math.PI) / WRIST_GEAR_RATIO;
    ConfigurationUtils.applyCheckRecordRev(
        () -> wristMotor.getEncoder().setPositionConversionFactor(relativeEncoderConversionFactor),
        () ->
            ConfigurationUtils.fpEqual(
                wristMotor.getEncoder().getPositionConversionFactor(),
                relativeEncoderConversionFactor),
        faultRecorder.run("Position conversion factor"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () ->
            wristMotor
                .getEncoder()
                .setVelocityConversionFactor(relativeEncoderConversionFactor / 60.0),
        () ->
            ConfigurationUtils.fpEqual(
                wristMotor.getEncoder().getVelocityConversionFactor(),
                relativeEncoderConversionFactor / 60.0),
        faultRecorder.run("Velocity conversion factor"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        wristMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        wristEventEntry::append,
        "Wrist motor",
        faultRecorder.getFaultString());
    wristAlert.set(faultRecorder.hasFault());
  }

  public double getPosition() {
    return MathUtil.angleModulus(
        Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition()) + WRIST_OFFSET);
  }

  public boolean atGoal() {
    return controller.atGoal();
  }

  public boolean atBottom() {
    return controller.getGoal().position == WRIST_MIN.getRadians() && atGoal();
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
    wristVoltageReq.append(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage)).withName("WristVoltage");
  }

  public Command setPositonCommand(Rotation2d desiredPosition) {
    return this.run(
            () -> {
              double feedbackOutput = controller.calculate(getPosition());
              TrapezoidProfile.State currentSetpoint = controller.getSetpoint();

              setVoltage(
                  feedbackOutput
                      + feedforward.calculate(currentSetpoint.position, currentSetpoint.velocity));
            })
        .beforeStarting(
            () -> {
              controller.reset(getPosition(), wristMotor.getEncoder().getVelocity());
              controller.setGoal(desiredPosition.getRadians());
            })
        .withName("SetWristPosition");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return wristSysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return wristSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    wristMotor.logValues();
    absoluteEncoderEntry.append(getPosition());
  }
}
