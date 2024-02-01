

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.ShooterConstants.*;

public class RunWristAndFlywheels extends ParallelCommandGroup {
  private final WristToPositionCommand wristtoposition;
  private final WristSubsystem wristSubsystem;
  private final double desiredPosition;
  private final ShooterSubsystem shooterSubsystem;
  public RunWristAndFlywheels(WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
   
    this.wristSubsystem = wristSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addCommands(new WristToPositionCommand(wristSubsystem, SHOOTING_ANGLE));
    addCommands(new shooterSubsystem.RunFlyRPM(SHOOTING_RPM));



  }
}
