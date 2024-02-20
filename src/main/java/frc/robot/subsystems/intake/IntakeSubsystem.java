// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.telemetry.types.EventTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;

public class IntakeSubsystem extends SubsystemBase {
  private static final Alert intakeMotorAlert =
      new Alert("Intake motor had a fault initializing", Alert.AlertType.ERROR);
  private final TelemetryCANSparkMax intakeMotor =
      new TelemetryCANSparkMax(
          INTAKE_MOTOR_ID,
          CANSparkLowLevel.MotorType.kBrushless,
          "/intake/motor",
          MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry intakeEventEntry = new EventTelemetryEntry("/intake/events");

  public IntakeSubsystem() {
    configMotor();
  }

  private void configMotor() {
    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    ConfigurationUtils.applyCheckRecordRev(
        () -> intakeMotor.setCANTimeout(250),
        () -> true,
        faultRecorder.run("CAN timeout"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        intakeMotor::restoreFactoryDefaults,
        () -> true,
        faultRecorder.run("Factory default"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> intakeMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
        () -> true,
        faultRecorder.run("Current limit"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecord(
        () -> intakeMotor.setInverted(INVERTED),
        () -> intakeMotor.getInverted() == INVERTED,
        faultRecorder.run("Invert"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        () -> intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
        () -> intakeMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
        faultRecorder.run("Idle mode"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordRev(
        intakeMotor::burnFlashWithDelay,
        () -> true,
        faultRecorder.run("Burn flash"),
        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        intakeEventEntry::append,
        "Intake motor",
        faultRecorder.getFaultString());
    intakeMotorAlert.set(faultRecorder.hasFault());
  }

  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    intakeMotor.logValues();
  }

  public Command checkIntakeCommand() {
    return this.run(() -> this.setIntakeVoltage(5));
  }

  public Command setIntakeVoltageCommand(double voltage) {
    return this.run(() -> this.setIntakeVoltage(voltage));
  }
}
