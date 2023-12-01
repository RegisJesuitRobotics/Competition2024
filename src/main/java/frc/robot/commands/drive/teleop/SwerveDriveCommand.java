package frc.robot.commands.drive.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TeleopConstants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.utils.RaiderMathUtils;
import frc.robot.utils.RaiderUtils;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveDriveCommand extends CommandBase {
    private static final Rotation2d oneHundredEightyDegrees = Rotation2d.fromDegrees(180);

    private final Supplier<Translation2d> translationSupplier;
    private final DoubleSupplier omegaRadiansSecondSupplier;
    private final BooleanSupplier isFieldRelativeSupplier;

    private final SwerveDriveSubsystem driveSubsystem;

    public SwerveDriveCommand(
            Supplier<Translation2d> translationSupplier,
            DoubleSupplier omegaRadiansSecondSupplier,
            BooleanSupplier isFieldRelativeSupplier,
            SwerveDriveSubsystem driveSubsystem) {
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

        Rotation2d currentHeading = driveSubsystem.getPose().getRotation();
        double omega = omegaRadiansSecondSupplier.getAsDouble();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), omega);

        if (isFieldRelative) {
            if (RaiderUtils.shouldFlip()) {
                currentHeading = currentHeading.plus(oneHundredEightyDegrees);
            }
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, currentHeading);
        }

        if (RaiderMathUtils.isChassisSpeedsZero(
                chassisSpeeds,
                TeleopConstants.MINIMUM_VELOCITY_METERS_SECOND,
                TeleopConstants.MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND)) {
            driveSubsystem.stopMovement();
        } else {
            driveSubsystem.setChassisSpeeds(chassisSpeeds, TeleopConstants.OPEN_LOOP_DRIVETRAIN);
        }
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
