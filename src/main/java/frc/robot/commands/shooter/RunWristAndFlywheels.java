package frc.robot.commands.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.wrist.WristToPositionCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class RunWristAndFlywheels extends ParallelCommandGroup {
  private final WristSubsystem wristSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public RunWristAndFlywheels(WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {

    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addCommands(new WristToPositionCommand(wristSubsystem, SHOOTING_ANGLE));
    addCommands(shooterSubsystem.RunFlyRPM(SHOOTING_RPM));
  }
}
