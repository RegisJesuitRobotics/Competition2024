public class IntakeToShooterCommand {
    public Command IntakeToShooterCommand(){
        return new ParallelCommandGroup();
    }
}
