// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterIntake extends Command {

  ShooterSubsystem shooterSubsystem;

  public RunShooterIntake(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
 
  }


  @Override
  public void initialize() {
  


  }


  @Override
  public void execute() {shooterSubsystem.runShooterTransportIn()}


  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.TransportStop()
    shooterSubsystem.setBothFlyVoltage(0);
    
  }


  @Override
  public boolean isFinished() {
    return shooterSubsystem.isAtSensor();
  }
}
