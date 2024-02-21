package frc.robot.commands.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSuperstructure;

public class intakeSlapCommand extends ParallelCommandGroup {
  private final IntakeSubsystem intakeSubsystem;
  private final SlapdownSuperstructure slapdownSuperstructure;

  public intakeSlapCommand(
      IntakeSubsystem intakeSubsystem, SlapdownSuperstructure slapdownSuperstructure) {
    this.intakeSubsystem = intakeSubsystem;
    this.slapdownSuperstructure = slapdownSuperstructure;

    addCommands(
        intakeSubsystem.setIntakeVoltageCommand(INTAKE_VOLTAGE));
//        slapdownSuperstructure.setFeederVoltageCommand(5));
  }
}
