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
import frc.robot.utils.RaiderUtils;

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
    boolean faultInitializing = false;
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> intakeMotor.setCANTimeout(250),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            intakeMotor::restoreFactoryDefaults,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> intakeMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            () -> intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
            () -> intakeMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
    faultInitializing |=
        RaiderUtils.applyAndCheckRev(
            intakeMotor::burnFlashWithDelay,
            () -> true,
            Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

    intakeEventEntry.append("Intake motor initialized" + (faultInitializing ? " with faults" : ""));
    intakeMotorAlert.set(faultInitializing);
  }

  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public Command setIntakeVoltageCommand(double voltage) {
    return this.run(() -> this.setIntakeVoltage(voltage));
  }
}
