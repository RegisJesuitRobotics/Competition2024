package frc.robot.commands.shooter;

import static frc.robot.Constants.TransportConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;


public class ShooterIntakeCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;

  public ShooterIntakeCommand(ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.transportSubsystem = transportSubsystem;

    addRequirements(shooterSubsystem);
    addRequirements(transportSubsystem);
  }

  @Override
  public void initialize() {
    transportSubsystem.runShooterTransportVoltage(TRANSPORT_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    transportSubsystem.runShooterTransportVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isAtSensor();
  }
}
