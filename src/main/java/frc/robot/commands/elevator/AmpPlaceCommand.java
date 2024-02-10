package frc.robot.commands.elevator;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.WristConstants.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.ShootAtAngleCommand;
import frc.robot.commands.wrist.WristToPositionCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class AmpPlaceCommand extends SequentialCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;

  public AmpPlaceCommand(
      ElevatorSubsystem elevatorSubsystem,
      WristSubsystem wristSubsystem,
      ShooterSubsystem shooterSubsystem,
      TransportSubsystem transportSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.wristSubsystem = wristSubsystem;
    this.transportSubsystem = transportSubsystem;
    addCommands(
        new ElevatorToPositionCommand(elevatorSubsystem, ELEVATOR_AMP_POS),
        new WristToPositionCommand(wristSubsystem, WRIST_AMP_POSITION),
        new ShootAtAngleCommand(
            shooterSubsystem, transportSubsystem, wristSubsystem, WRIST_AMP_POSITION));
  }
}
