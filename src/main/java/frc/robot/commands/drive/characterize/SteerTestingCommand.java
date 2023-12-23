package frc.robot.commands.drive.characterize;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.tunable.gains.TunableDouble;

public class SteerTestingCommand extends Command {
  private final SwerveDriveSubsystem driveSubsystem;
  private final TunableDouble desiredSteer =
      new TunableDouble("/char/desiredSteerTesting", 0.0, true);

  public SteerTestingCommand(SwerveDriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Rotation2d angle = Rotation2d.fromDegrees(desiredSteer.get());

    SwerveModuleState[] states = new SwerveModuleState[SwerveConstants.NUM_MODULES];

    for (int i = 0; i < SwerveConstants.NUM_MODULES; i++) {
      states[i] = new SwerveModuleState(0.0, angle);
    }

    driveSubsystem.setRawStates(true, true, states);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopMovement();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
