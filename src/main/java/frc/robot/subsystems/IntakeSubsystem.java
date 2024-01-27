// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
  private TelemetryCANSparkMax sparkMax = new TelemetryCANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless, "intake/motors", true);
  private DigitalInput intakeSensor = new DigitalInput(SENSOR_ID);

  public void motorVoltage(){
    sparkMax.setVoltage(3);
  }

  public void motorSpeed(){
    sparkMax.set(5)
  }

public boolean atIntakeSensor(){
    return intakeSensor.get();
}

  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
