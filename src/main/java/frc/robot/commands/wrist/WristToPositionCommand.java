package frc.robot.commands.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.WristSubsystem;
import static frc.robot.Constants.WristConstants.*;

public class WristToPositionCommand extends Command {
    private final WristSubsystem wristSubsystem;
    private final double desiredPosition;
    public WristToPositionCommand(WristSubsystem wristSubsystem, Rotation2d desiredPosition){
        this.wristSubsystem = wristSubsystem;

        addRequirements(wristSubsystem);

        this.desiredPosition = MathUtil.clamp(desiredPosition.getRadians(), WRIST_LOW.getRadians(), WRIST_HIGH.getRadians());

    }

    @Override
    public void initialize(){
        wristSubsystem.setDesiredPosition(Rotation2d.fromRadians(desiredPosition));
    }

    @Override
    public void end(boolean interrupted){
        wristSubsystem.stopMovement();
    }

    @Override
    public boolean isFinished(){
        return wristSubsystem.atGoal();
    }
}
