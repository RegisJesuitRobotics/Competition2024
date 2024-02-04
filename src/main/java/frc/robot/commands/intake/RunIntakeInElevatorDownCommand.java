package frc.robot.commands.intake;

import static frc.robot.Constants.TransportConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.ElevatorToPositionCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;

public class RunIntakeInElevatorDownCommand extends ParallelCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final TransportSubsystem transportSubsystem;

  public RunIntakeInElevatorDownCommand(
      ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem,
      TransportSubsystem transportSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.transportSubsystem = transportSubsystem;

    addCommands(
        new ElevatorToPositionCommand(elevatorSubsystem, 0),
        new RunIntakeInCommand(intakeSubsystem));
    addCommands(transportSubsystem.runTransportCommand());
  }
}
