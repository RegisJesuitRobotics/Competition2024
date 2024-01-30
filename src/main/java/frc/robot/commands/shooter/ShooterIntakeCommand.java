package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterIntakeCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem; 

  public ShooterIntakeCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setFlyVoltage(INTAKE_VOLTAGE);
    shooterSubsystem.runShooterTransportVoltage();
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlyVoltage(0);
    shooterSubsystem.shooterTransportStop();
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isAtSensor();
  }
}
