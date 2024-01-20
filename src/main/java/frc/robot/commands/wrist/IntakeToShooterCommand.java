package frc.robot.commands.wrist;

import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RunIntakeInElevatorDownCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class IntakeToShooterCommand extends SequentialCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public IntakeToShooterCommand(
      ElevatorSubsystem elevatorSubsystem,
      IntakeSubsystem intakeSubsystem,
      WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;

    addRequirements(intakeSubsystem);
    addRequirements(elevatorSubsystem);
    addRequirements(wristSubsystem);
    addRequirements(shooterSubsystem);

    addCommands(new WristToPositionCommand(wristSubsystem, WRIST_LOW));
    addCommands(
        new RunIntakeInElevatorDownCommand(elevatorSubsystem, intakeSubsystem, shooterSubsystem));
  }
}
