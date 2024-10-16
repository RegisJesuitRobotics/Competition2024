package frc.robot.commands.drive.teleop;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveDriveCommand extends Command {
  private final Supplier<Translation2d> translationSupplier;
  private final DoubleSupplier omegaRadiansSecondSupplier;
  private final BooleanSupplier isFieldRelativeSupplier;

  private final CommandSwerveDrivetrain driveSubsystem;

  private final RobotCentric robotCentric = new SwerveRequest.RobotCentric();
  private final FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
  private final Idle idleRequest = new SwerveRequest.Idle();

  public SwerveDriveCommand(
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier omegaRadiansSecondSupplier,
      BooleanSupplier isFieldRelativeSupplier,
      CommandSwerveDrivetrain driveSubsystem) {
    this.translationSupplier = translationSupplier;
    this.omegaRadiansSecondSupplier = omegaRadiansSecondSupplier;
    this.isFieldRelativeSupplier = isFieldRelativeSupplier;

    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean isFieldRelative = isFieldRelativeSupplier.getAsBoolean();
    Translation2d translation = translationSupplier.get();

    if (isFieldRelative) {
      driveSubsystem.applyRequest(
          () ->
              fieldCentricRequest
                  .withVelocityX(translation.getX())
                  .withVelocityY(translation.getY())
                  .withRotationalRate(omegaRadiansSecondSupplier.getAsDouble()));
    } else {
      driveSubsystem.applyRequest(
          () ->
              robotCentric
                  .withVelocityX(translation.getX())
                  .withVelocityY(translation.getY())
                  .withRotationalRate(omegaRadiansSecondSupplier.getAsDouble()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.applyRequest(() -> idleRequest);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
