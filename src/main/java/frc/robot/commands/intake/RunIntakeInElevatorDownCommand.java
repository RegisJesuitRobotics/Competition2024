package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.ElevatorToPositionCommand;
import frc.robot.commands.shooter.ShooterIntakeCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunIntakeInElevatorDownCommand extends ParallelCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public RunIntakeInElevatorDownCommand(
      ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(elevatorSubsystem);
    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);

    addCommands(new ElevatorToPositionCommand(elevatorSubsystem, 0));
    addCommands(new RunIntakeInCommand(intakeSubsystem));
    addCommands(new ShooterIntakeCommand(shooterSubsystem));
  }
}
