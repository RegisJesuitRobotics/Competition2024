package frc.robot.commands.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.wrist.WristToPositionCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ElevatorWristParallelCommand extends ParallelCommandGroup {
  private final WristSubsystem wristSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorWristParallelCommand(
      WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;

    addCommands(
        new WristToPositionCommand(wristSubsystem, WRIST_LOW),
        new ElevatorToPositionCommand(elevatorSubsystem, 0));
  }
}
