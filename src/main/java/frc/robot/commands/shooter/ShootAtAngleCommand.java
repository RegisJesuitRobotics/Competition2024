public class ShootAtAngleCommand extends SequentialCommandGroup{

    public final ShooterSubsystem shooterSubsystem;
    public final TransportSubsystem transportSubsystem;
    public final WristSubsystem wristSubsystem;
    private final RunWristAndFlywheels runWristAndFlyWheels;

    public ShootAtAngleCommand(ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, WristSubsystem wristSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.transportSubsystem = transportSubsystem;
        this.wristSubsystem = wristSubsystem;

        addCommands(new RunWristAndFlywheels());
        addCommands(new transportSubsystem.runTransportCommand());
        addCommands(new shooterSubsystem.RunFlyRPM(0));
        
    }
    
}
