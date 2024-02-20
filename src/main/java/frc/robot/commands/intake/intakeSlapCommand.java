package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.slapdown.SlapdownSubsystem;
import static frc.robot.Constants.IntakeConstants.*;

public class intakeSlapCommand extends ParallelCommandGroup {
    private final IntakeSubsystem intakeSubsystem;
    private final SlapdownSubsystem slapdownSubsystem;
    public intakeSlapCommand(IntakeSubsystem intakeSubsystem, SlapdownSubsystem slapdownSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        this.slapdownSubsystem = slapdownSubsystem;

        addCommands(intakeSubsystem.setIntakeVoltageCommand(INTAKE_VOLTAGE),
                slapdownSubsystem.setFeederVoltageCommand(5));

    }
}
