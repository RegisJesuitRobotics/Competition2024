// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeInCommand extends Command {
 
  private final IntakeSubsystem intakesubsystem;

  public RunIntakeInCommand(IntakeSubsystem intakesubsystem) {

    this.intakesubsystem = intakesubsystem;
    addRequirements(intakesubsystem);
  }

  
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    intakesubsystem.motorVoltage();
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return intakesubsystem.atIntakeSensor();
  }
}
