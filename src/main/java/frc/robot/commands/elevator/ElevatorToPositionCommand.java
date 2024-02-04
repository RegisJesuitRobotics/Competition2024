package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToPositionCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double desiredPosition;

  public ElevatorToPositionCommand(ElevatorSubsystem elevatorSubsystem, double desiredPosition) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.desiredPosition = desiredPosition;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setDesiredPosition(desiredPosition);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopMove();
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atGoal();
  }
}
