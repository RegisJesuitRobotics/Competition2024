package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.telemetry.types.DoubleTelemetryEntry;

import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterization extends Command {



// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

//    private static final  characterizationSpeed =
//            new LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);

    private double characterizationSpeed;
    private DoubleTelemetryEntry drivePosLog = new DoubleTelemetryEntry("drive/RadiusCharacterization/DrivePosition", true);
    private DoubleTelemetryEntry accumGyroYawRadsLog = new DoubleTelemetryEntry("drive/RadiusCharacterization/AccumGyroYawRads", true);
    private DoubleTelemetryEntry currentEffectiveWheelRadiusLog = new DoubleTelemetryEntry("drive/RadiusCharacterization/CurrentWheelRadiusInches", true);

    private static final double driveRadius = Constants.SwerveConstants.MODULE_TRANSLATIONS[0].getNorm();
    private static DoubleSupplier gyroYawRadsSupplier;

//    RobotState.getInstance().getOdometryPose().getRotation().getRadians()


    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int value;

        private Direction(int value) {
            this.value = value;
        }
    }

    private final SwerveDriveSubsystem drive;
    private final Direction omegaDirection;
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;

    private double[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(SwerveDriveSubsystem drive, Direction omegaDirection, double speed) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        characterizationSpeed = speed;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        gyroYawRadsSupplier = ()-> drive.getGyroYaw().getRadians();

        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;
        startWheelPositions = drive.getWheelRadiusCharPosition();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        drive.runWheelCharacterization(
                omegaLimiter.calculate(omegaDirection.value * characterizationSpeed));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        double averageWheelPosition = 0.0;
        double[] wheelPositiions = drive.getWheelRadiusCharPosition();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;

        drivePosLog.append(averageWheelPosition);
        accumGyroYawRadsLog.append(accumGyroYawRads);
        currentEffectiveWheelRadiusLog.append(Units.metersToInches(currentEffectiveWheelRadius));
    }

    @Override
    public void end(boolean interrupted) {
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        } else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }
}
