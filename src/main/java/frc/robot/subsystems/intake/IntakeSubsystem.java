// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  private final TelemetryCANSparkMax intakeMotor =
      new TelemetryCANSparkMax(
          INTAKE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "intake/motors", true);

  private final DigitalInput intakeSensor = new DigitalInput(INTAKE_SENSOR_ID);

  public IntakeSubsystem() {
    intakeMotor.setInverted(true);
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
