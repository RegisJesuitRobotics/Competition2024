package frc.robot.commands.drive.elevator;

import static frc.robot.Constants.ElevatorConstants.ELEVATOR_LIMIT_SWITCH;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToPositionCommand extends Command {

  private final DigitalInput elevatorLimitSwitch = new DigitalInput(ELEVATOR_LIMIT_SWITCH);
  ElevatorSubsystem subsystem;
  double position;

  public ElevatorToPositionCommand(ElevatorSubsystem subsystem, double position) {
    this.subsystem = subsystem;
    this.position = position;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    if (!elevatorLimitSwitch.get()) {
      subsystem.setDesiredPosition(position);

    } else {
      subsystem.stopMove();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    subsystem.stopMove();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
