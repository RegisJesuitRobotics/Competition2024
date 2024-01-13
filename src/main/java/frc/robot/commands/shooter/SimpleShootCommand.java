package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SimpleShootCommand extends Command {
    ShooterSubsystem shooterSubsystem;
    public SimpleShootCommand(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize(){
        shooterSubsystem.setRPM(2000);
    }


}
