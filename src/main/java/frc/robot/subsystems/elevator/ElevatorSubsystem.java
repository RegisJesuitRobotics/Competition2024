package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class ElevatorSubsystem extends SubsystemBase {

  public static int instances = 0;

  private final Alert leftMotorFaultAlert, rightMotorFaultAlert;
  private StatusSignal<Double> leftPositionSignal, leftVelocitySignal;
  private StatusSignal<Double> rightPositionSignal, rightVelocitySignal;
  private final TelemetryTalonFX leftMotor =
      new TelemetryTalonFX(ElevatorConstants.LEFT_ELEVATOR_MOTOR, "elevator/left", true);

  private final TelemetryTalonFX rightMotor =
      new TelemetryTalonFX(ElevatorConstants.RIGHT_ELEVATOR_MOTOR, "elevator/right", true);

  private final TunableTelemetryProfiledPIDController controller =
      new TunableTelemetryProfiledPIDController(
          "elevator/controller", PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);

  private SimpleMotorFeedforward feedforward = FF_GAINS.createFeedforward();

  private final DigitalInput bottomLimit = new DigitalInput(ELEVATOR_LIMIT_SWITCH);

  public ElevatorSubsystem() {
    int instanceID = instances++;
    leftMotorFaultAlert = new Alert("Module " + instanceID + ": ", Alert.AlertType.ERROR);
    rightMotorFaultAlert = new Alert("Module " + instanceID + ": ", Alert.AlertType.ERROR);
    configMotors();
  }

  private void configMotors() {
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = CONTINUOUS_CURRENT_LIMIT;
    motorConfiguration.CurrentLimits.SupplyCurrentThreshold = PEAK_CURRENT_LMIT;
    motorConfiguration.CurrentLimits.SupplyTimeThreshold = PEAK_CURRENT_LIMIT_SECONDS;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftPositionSignal = leftMotor.getPosition();
    leftVelocitySignal = leftMotor.getVelocity();

    rightPositionSignal = rightMotor.getPosition();
    rightVelocitySignal = rightMotor.getVelocity();

    boolean faultInitializingRight = false;
    boolean faultInitializingLeft = false;

    double conversionFactor = METERS_PER_REV / ELEVATOR_GEAR_RATIO;



    faultInitializingLeft |=
        RaiderUtils.applyAndCheckCTRE(
            () -> leftMotor.getConfigurator().apply(motorConfiguration),
            () -> {
              TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
              leftMotor.getConfigurator().refresh(appliedConfig);
              return appliedConfig.equals(motorConfiguration);
            },
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializingRight |=
        RaiderUtils.applyAndCheckCTRE(
            () -> rightMotor.getConfigurator().apply(motorConfiguration),
            () -> {
              TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
              rightMotor.getConfigurator().refresh(appliedConfig);
              return appliedConfig.equals(motorConfiguration);
            },
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);


    leftMotor.hasResetOccurred();
    rightMotor.hasResetOccurred();

    leftMotor.setLoggingPositionConversionFactor(conversionFactor);
    rightMotor.setLoggingPositionConversionFactor(conversionFactor);

    leftMotor.setLoggingVelocityConversionFactor(conversionFactor / 60);
    rightMotor.setLoggingVelocityConversionFactor(conversionFactor / 60);


    leftMotorFaultAlert.set(faultInitializingLeft);
    rightMotorFaultAlert.set(faultInitializingRight);
  }

  public void atBottomLimit() {
    if (bottomLimit.get()) {
      leftMotor.setPosition(0);
      rightMotor.setPosition(0);
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
    leftMotor.setPosition(position);
    rightMotor.setPosition(position);
  }

  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  public double getPosition() {
    return leftMotor.getPosition().getValue();
  }

  public void stopMove() {
    rightMotor.setVoltage(0);
    leftMotor.setVoltage(0);
  }

  public Command runElevatorCommand(double voltage) {
    voltage = MathUtil.clamp(voltage, -0.5, 0.5);

    if (getPosition() > ELEVATOR_MIN && getPosition() < ELEVATOR_MAX) {
      double finalVoltage = voltage;
      return this.runEnd(() -> this.setVoltage(finalVoltage), () -> this.setVoltage(0));

    } else {
      return this.run(() -> this.setVoltage(0));
    }
  }

  public Command setElevatorPositionCommand(double position) {
    return this.run(() -> this.setDesiredPosition(position));
  }

  @Override
  public void periodic() {
    double feedbackOutput = controller.calculate(getPosition());

    TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
    double combinedOutput = feedbackOutput + feedforward.calculate(currentSetpoint.velocity);
    leftMotor.setVoltage(combinedOutput);
    rightMotor.setVoltage(combinedOutput);
    atBottomLimit();
    logValues();
  }

  private void logValues() {
    leftMotor.logValues();
    rightMotor.logValues();
    if (FF_GAINS.hasChanged()) {
      feedforward = FF_GAINS.createFeedforward();
    }
  }
}
