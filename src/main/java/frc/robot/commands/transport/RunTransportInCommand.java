package frc.robot.commands.transport;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transport.TransportSubsystem;
import static frc.robot.Constants.TransportConstants.*;

public class RunTransportInCommand extends Command {
    private final TransportSubsystem transportSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    public RunTransportInCommand(TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem){
        this.transportSubsystem = transportSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(transportSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize(){
        transportSubsystem.runShooterTransportVoltage(TRANSPORT_VOLTAGE);
    }
    @Override
    public void execute(){}

    @Override
    public void end(boolean interrupted){
        transportSubsystem.runShooterTransportVoltage(0);
    }
    @Override
    public boolean isFinished(){
       return shooterSubsystem.AtSensor();
    }


}
