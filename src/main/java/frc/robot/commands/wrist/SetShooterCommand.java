// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterCommand extends Command {

  WristSubsystem wristsubsystem;

  public SetShooterCommand(WristSubsystem wristsubsystem) {
    this.wristsubsystem = wristsubsystem;
  }

  
  @Override
  public void initialize() {
    wristsubsystem.setVoltage(.5);
  }

 
  @Override
  public void execute() {

    boolean bottom;
    
    if (atTransportAngle == true){
      wristsubsystem.stopMovement();
      getPosition.get() == bottom;
    }
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
