// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


public class DownandUpCommand extends Command {
  
  WristSubsystem wristsubsystem;


  public DownandUpCommand(WristSubsystem wristsubsystem) {
    this.wristsubsystem = wristsubsystem;
  }

  @Override
  public void initialize() {
  }

  
  @Override
  public void execute() {
    if (isAtSensor == true){
      setDesiredPosition.set(0);
      setDesiredPosition.set(Set_Shooter_Angle);
    }
  }

 
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
