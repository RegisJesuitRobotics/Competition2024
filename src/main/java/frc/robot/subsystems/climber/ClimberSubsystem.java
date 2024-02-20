package frc.robot.subsystems.climber;

import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.DoubleTelemetryEntry;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryTalonFX;
import frc.robot.utils.ConfigEquality;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class ClimberSubsystem extends SubsystemBase {
  private final TelemetryTalonFX climberMotor =
      new TelemetryTalonFX(CLIMBER_MOTOR_ID, "/climber/motor", MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry eventEntry = new EventTelemetryEntry("/climber/events");
  private final DoubleTelemetryEntry voltageReqEntry =
      new DoubleTelemetryEntry("/climber/voltageReq", MiscConstants.TUNING_MODE);

  public ClimberSubsystem() {
    configMotor();

    setDefaultCommand(setVoltageCommand(0.0));
  }

  // TODO: Make it move with the elevator
  private void configMotor() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = SUPPLY_CURRENT_THRESHOLD;
    motorConfig.CurrentLimits.SupplyTimeThreshold = SUPPLY_TIME_THRESHOLD;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.MotorOutput.Inverted =
        INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> climberMotor.getConfigurator().apply(motorConfig),
        () -> {
          TalonFXConfiguration appliedConfig = new TalonFXConfiguration();
          climberMotor.getConfigurator().refresh(appliedConfig);
          return ConfigEquality.isTalonConfigurationEqual(motorConfig, appliedConfig);
        },
        faultRecorder.run("Motor config"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        climberMotor::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        eventEntry::append,
        "Climber motor",
        faultRecorder.getFaultString());
  }

  public void setVoltage(double voltage) {
    climberMotor.setControl(new VoltageOut(voltage));
    voltageReqEntry.append(voltage);
  }

  public Command setVoltageCommand(double voltage) {
    return this.run(() -> setVoltage(voltage));
  }
}
