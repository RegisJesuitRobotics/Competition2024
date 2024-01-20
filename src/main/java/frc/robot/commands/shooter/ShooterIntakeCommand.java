package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterIntakeCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    public ShooterIntakeCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){
        shooterSubsystem.setBothFlyVoltage(INTAKE_VOLTAGE);
        shooterSubsystem.runShooterTransportIn();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.setBothFlyVoltage(0);
        shooterSubsystem.shooterTransportStop();
    }

    @Override
    public boolean isFinished(){
        return shooterSubsystem.isAtSensor();
    }
}
