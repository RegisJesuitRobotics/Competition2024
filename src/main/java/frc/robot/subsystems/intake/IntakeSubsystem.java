// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.SlapdownConstants.FREE_MOTOR_CURRENT;
import static frc.robot.Constants.SlapdownConstants.STALL_MOTOR_CURRENT;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

public class IntakeSubsystem extends SubsystemBase {

  private Alert intakeMotorAlert;
  private final TelemetryCANSparkMax intakeMotor =
      new TelemetryCANSparkMax(
          INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "intake/motors", true);

  private final DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);

  public IntakeSubsystem() {
    intakeMotor.setInverted(true);
    intakeMotorAlert = new Alert("Intake Motor: ", Alert.AlertType.ERROR);
  }

  private void motorConfig() {

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

    intakeMotorAlert.set(faultInitializing);
  }

  public void runIntakeIn() {

    intakeMotor.setVoltage(INTAKE_VOLTAGE);
  }

  public boolean atIntakeSensor() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
