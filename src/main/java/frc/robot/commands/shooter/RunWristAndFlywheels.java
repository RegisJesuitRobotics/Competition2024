package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RunWristAndFlywheels extends ParallelCommandGroup {
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public RunWristAndFlywheels(
      WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem, Rotation2d shootingAngle) {

    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addCommands(
        wristSubsystem.setPositonCommand(shootingAngle),
        shooterSubsystem.runVelocityCommand(SHOOTING_RPM));
  }
}
