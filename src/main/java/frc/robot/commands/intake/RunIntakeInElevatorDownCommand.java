package frc.robot.commands.intake;

import static frc.robot.Constants.TransportConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.transport.RunTransportInCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;

public class RunIntakeInElevatorDownCommand extends ParallelCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public RunIntakeInElevatorDownCommand(
      ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem,
      TransportSubsystem transportSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.transportSubsystem = transportSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addCommands(new RunIntakeInCommand(intakeSubsystem));
    addCommands(new RunTransportInCommand(transportSubsystem, shooterSubsystem));
  }
}
