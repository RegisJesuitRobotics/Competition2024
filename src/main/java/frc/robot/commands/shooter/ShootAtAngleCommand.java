package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.RunWristAndFlywheels;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ShootAtAngleCommand extends SequentialCommandGroup {

    public final ShooterSubsystem shooterSubsystem;
    public final TransportSubsystem transportSubsystem;
    public final WristSubsystem wristSubsystem;


    public ShootAtAngleCommand(ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, WristSubsystem wristSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.transportSubsystem = transportSubsystem;
        this.wristSubsystem = wristSubsystem;

        addCommands(new RunWristAndFlywheels(wristSubsystem, shooterSubsystem));
        addCommands(transportSubsystem.runTransportCommand());
        addCommands(shooterSubsystem.RunFlyRPM(0));
        
    }
    
}
