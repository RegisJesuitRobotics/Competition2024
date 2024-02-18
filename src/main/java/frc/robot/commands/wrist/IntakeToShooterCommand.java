package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorWristParallelCommand;
import frc.robot.commands.intake.runIntakeTransportCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class IntakeToShooterCommand extends SequentialCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final WristSubsystem wristSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public IntakeToShooterCommand(
      ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem,
      WristSubsystem wristSubsystem,
      TransportSubsystem transportSubsystem,
      ShooterSubsystem shooterSubsystem) {

    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.transportSubsystem = transportSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addCommands(
        new ElevatorWristParallelCommand(wristSubsystem, elevatorSubsystem),
        new runIntakeTransportCommand(
            elevatorSubsystem, intakeSubsystem, transportSubsystem, shooterSubsystem));
  }
}
