package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.Alert;

public class SimpleShootCommand extends Command {
  private final Alert notAtSensorAlert =
      new Alert("Frisbee not at Shooter", Alert.AlertType.WARNING);
  ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem; 

  public SimpleShootCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    if (shooterSubsystem.isAtSensor()) {
      shooterSubsystem.runShooterTransportVoltage();
      shooterSubsystem.setRPM(2000);
    } else {
      notAtSensorAlert.set(!shooterSubsystem.isAtSensor());
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlyVoltage(0);
    shooterSubsystem.TransportStop();
  }

  @Override
  public boolean isFinished() {
    return shooterSubsystem.isAtSensor();
  }
}
