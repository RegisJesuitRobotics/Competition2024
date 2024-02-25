package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.telemetry.tunable.gains.TunableArmElevatorFFGains;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.tunable.gains.TunableTrapezoidalProfileGains;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;

/** File containing all constants for the robot. */
public final class Constants {
  private Constants() {}

  public static class ScoringConstants {
    public static final double AMP_ELEVATOR_HEIGHT = Units.inchesToMeters(3.5);
    public static final double AMP_WRIST_ANGLE_RADIANS = Units.degreesToRadians(105.0);
  }

  public static class IntakeConstants {
    public static final double INTAKE_VOLTAGE = 5.5;
    public static final int INTAKE_MOTOR_ID = 6;
    public static final boolean INVERTED = false;

    public static final int STALL_MOTOR_CURRENT = 40;
    public static final int FREE_MOTOR_CURRENT = 20;
  }

  public static class SlapdownConstants {
    public static final int FEEDER_MOTOR_ID = 7;
    public static final int ROTATION_MOTOR_ID = 9;

    public static final boolean FEEDER_INVERTED = true;
    public static final boolean ROTATION_INVERTED = false;

    public static final double ROTATION_GEAR_RATIO = 5.0 * 3.0 * 30.0 / 15.0;
    public static final double FEEDER_GEAR_RATIO = 5.0;

    public static final int FEED_STALL_MOTOR_CURRENT = 20;
    public static final int FEED_FREE_MOTOR_CURRENT = 10;

    public static final int ROTATION_STALL_MOTOR_CURRENT = 20;
    public static final int ROTATION_FREE_MOTOR_CURRENT = 10;

    public static final double ROTATION_UP_ANGLE = -0.0648 - Units.degreesToRadians(90);
    public static final double ROTATION_DOWN_ANGLE = 2.25 - Units.degreesToRadians(90);

    public static final int ROTATION_LIMIT_SWITCH_ID = 1;

    public static final double FEEDER_VOLTAGE = (6);

    public static final TunablePIDGains ROTATION_GAINS =
        new TunablePIDGains("/slapdown/rotation/gains", 4.0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains ROTATION_TRAP_GAINS =
        new TunableTrapezoidalProfileGains(
            "/slapdown/rotation/trapGains", 25, 40, MiscConstants.TUNING_MODE);
    public static final TunableArmElevatorFFGains ROTATION_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/slapdown/rotation/FFGains",
            0.10403,
            0.17546,
            0.61704,
            0.084257,
            MiscConstants.TUNING_MODE);
  }

  public static class ElevatorConstants {
    public static final boolean INVERTED = false;

    public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(10.5);
    public static final double ELEVATOR_MIN_HEIGHT = Units.inchesToMeters(0.0);

    // TODO: Figure these out
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;

    public static final int ELEVATOR_MOTOR_ID = 4;

    public static final int ELEVATOR_LIMIT_SWITCH = 0;

    public static final double ELEVATOR_GEAR_RATIO = 5.0 * 3.0;
    public static final double METERS_PER_REV =
        Units.inchesToMeters((Math.PI * 1.75) / (ELEVATOR_GEAR_RATIO));

    // TODO: Do These PID GAINS
    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("gains/elevator", 20.0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/extension",
            Units.inchesToMeters(28),
            Units.inchesToMeters(125),
            MiscConstants.TUNING_MODE);

    // TODO: TUNE FF GAINS
    public static final TunableArmElevatorFFGains FF_GAINS =
        new TunableArmElevatorFFGains(
            "gains/elevator", 0.22339, 0.041616, 13.1, 1.2556, MiscConstants.TUNING_MODE);
  }

  public static class SwerveConstants {
    private SwerveConstants() {}

    public static final int PIGEON_ID = 21;

    public static final int NUM_MODULES = 4;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3.875);
    public static final double DRIVE_GEAR_REDUCTION = (50.0 / 14) * (17.0 / 27) * (45.0 / 15);

    public static final double STEER_GEAR_REDUCTION = 150.0 / 7.0;

    public static final double DRIVE_PEAK_CURRENT_LIMIT = 65.0;
    public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT = 35.0;
    public static final double DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS = 0.2;
    public static final int STEER_STALL_CURRENT_LIMIT = 45;
    public static final int STEER_FREE_CURRENT_LIMIT = 25;

    // 0.47
    public static final TunablePIDGains DRIVE_VELOCITY_PID_GAINS =
        new TunablePIDGains("/gains/drive", 0.2, 0.0, 0.0, MiscConstants.TUNING_MODE);

    public static final TunableFFGains DRIVE_VELOCITY_FF_GAINS =
        new TunableFFGains("/gains/drive", 0.064022, 1.9843, 0.9255, MiscConstants.TUNING_MODE);

    public static final TunablePIDGains STEER_POSITION_PID_GAINS =
        new TunablePIDGains("/gains/steer", 1.0, 0.0, 0.1, MiscConstants.TUNING_MODE);

    // Left right distance between center of wheels
    public static final double TRACKWIDTH_METERS = Units.inchesToMeters(22.75);

    // Front back distance between center of wheels
    public static final double WHEELBASE_METERS = Units.inchesToMeters(21.75);

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)
        };

    public static final double MAX_VELOCITY_METERS_SECOND = 4.023;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_SECOND = 10.063;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED = 52.717;

    public static final int ODOMETRY_FREQUENCY = 200;

    public static final String CAN_BUS = "canivore";
    private static final SharedSwerveModuleConfiguration SHARED_SWERVE_MODULE_CONFIGURATION =
        new SharedSwerveModuleConfiguration(
            CAN_BUS,
            DRIVE_GEAR_REDUCTION,
            STEER_GEAR_REDUCTION,
            DRIVE_PEAK_CURRENT_LIMIT,
            DRIVE_CONTINUOUS_CURRENT_LIMIT,
            DRIVE_CONTINUOUS_CURRENT_LIMIT_TIME_SECONDS,
            STEER_FREE_CURRENT_LIMIT,
            STEER_STALL_CURRENT_LIMIT,
            WHEEL_DIAMETER_METERS,
            ODOMETRY_FREQUENCY,
            DRIVE_VELOCITY_PID_GAINS,
            DRIVE_VELOCITY_FF_GAINS,
            STEER_POSITION_PID_GAINS);

    public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            12, 8, 17, true, true, 2.862408, false, SHARED_SWERVE_MODULE_CONFIGURATION);
    public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            13, 5, 18, true, true, -1.294680, false, SHARED_SWERVE_MODULE_CONFIGURATION);

    public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            14, 10, 19, true, true, 0.368155, false, SHARED_SWERVE_MODULE_CONFIGURATION);

    public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            15, 3, 20, true, true, 2.906894, false, SHARED_SWERVE_MODULE_CONFIGURATION);
  }

  public static class AutoConstants {
    private AutoConstants() {}

    public static final double MAX_AUTO_VELOCITY_METERS_SECOND = 2.5;
    public static final double MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;

    public static final double MAX_AUTO_ANGULAR_VELOCITY_RADIANS_SECOND = Math.PI * 2;
    public static final double MAX_AUTO_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED = Math.PI;

    public static final TunablePIDGains TRANSLATION_POSITION_GAINS =
        new TunablePIDGains("/gains/driveXY", 2.0, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRANSLATION_POSITION_TRAPEZOIDAL_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/driveXY",
            MAX_AUTO_VELOCITY_METERS_SECOND,
            MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED,
            MiscConstants.TUNING_MODE);
    public static final TunablePIDGains ANGULAR_POSITION_PID_GAINS =
        new TunablePIDGains("/gains/driveAngular", 1.1, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains ANGULAR_POSITION_TRAPEZOIDAL_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/driveAngular",
            MAX_AUTO_ANGULAR_VELOCITY_RADIANS_SECOND,
            MAX_AUTO_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED,
            MiscConstants.TUNING_MODE);
  }

  public static class WristConstants {
    public static final double WRIST_GEAR_RATIO = 25.0 * 42.0 / 18.0;

    public static final Rotation2d WRIST_AMP_POSITION = new Rotation2d(Units.degreesToRadians(55));

    // TODO FIND WRIST CLAMPS
    public static final Rotation2d WRIST_MAX = new Rotation2d(Units.degreesToRadians(60));
    public static final Rotation2d WRIST_MIN = Rotation2d.fromRadians(-0.027);

    public static final double WRIST_OFFSET = -0.288 + Units.degreesToRadians(90.0);
    public static final int WRIST_ENCODER_PORT = 2;

    public static final int WRIST_MOTOR_ID = 2;
    public static final boolean INVERTED = true;
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;

    // TODO: TUNE PID & TRAP & FF
    // New gains, 0.43784, 0.0383, 0.76791, 0.55677
    public static final TunableArmElevatorFFGains WRIST_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/wrist/ffGains", .16624, 0.35068, 1.0512, 0.2769, MiscConstants.TUNING_MODE);
    //    public static final TunablePIDGains WRIST_PID_GAINS =
    //        new TunablePIDGains("/wrist/pidGains", 56.599, 0, 4.8897, MiscConstants.TUNING_MODE);
    //
    public static final TunablePIDGains WRIST_PID_GAINS =
        new TunablePIDGains("/wrist/pidGains", 2, 0, 0, MiscConstants.TUNING_MODE);

    public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
        new TunableTrapezoidalProfileGains(
            "/wrist/trapGains",
            Units.rotationsToRadians(1.5),
            Units.rotationsToRadians(1.5),
            MiscConstants.TUNING_MODE);
  }

  public static class TransportConstants {
    public static final int TRANSPORT_MOTOR_ID = 1;
    public static final boolean INVERTED = false;
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;
    public static final double TRANSPORT_LOAD_VOLTAGE = 4.0;

    public static final double TRANSPORT_CLOSE_SPEAKER_VOLTAGE = 12;
    public static final int SHOOTER_SENSOR_ID = 8;
  }

  public static class ShooterConstants {
    public static final int FREE_MOTOR_CURRENT = 25;
    public static final int STALL_MOTOR_CURRENT = 80;

    public static final boolean INVERTED = false;
    public static final int SHOOTER_ID = 11;

    public static final double SHOOTING_RPM = 3000.0;

    public static final Rotation2d SHOOTING_ANGLE =
        new Rotation2d(Units.degreesToRadians(10)); // TODO: Implement Ollies stuff

    public static final double SHOOTER_GEAR_RATIO = 1.0 / 2.0;

    public static TunablePIDGains SHOOTER_PID_GAINS =
        new TunablePIDGains("/shooter/pid", 0.005, 0, 0, MiscConstants.TUNING_MODE);
    public static TunableFFGains SHOOTER_FF_GAINS =
        new TunableFFGains("/shooter/FF", 0.39383, 0.0089833, 0.0010833, MiscConstants.TUNING_MODE);
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 16;
    public static final boolean INVERTED = false;

    public static final double SUPPLY_CURRENT_LIMIT = 65;
    public static final double SUPPLY_CURRENT_THRESHOLD = 40;
    public static final double SUPPLY_TIME_THRESHOLD = 0.0;
  }

  public static class TeleopConstants {
    private TeleopConstants() {}

    public static final boolean OPEN_LOOP_DRIVETRAIN = true;
    public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED = 15.0;
    public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED = 8.0 * Math.PI;
    public static final double MINIMUM_VELOCITY_METERS_SECOND = 0.10;
    public static final double MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND = 0.10;
  }

  public static class VisionConstants {
    private VisionConstants() {}

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            -8.017, 0, 27.088, new Rotation3d(0, Units.degreesToRadians(180 - 27.088), 0));
  }

  public static class MiscConstants {

    private MiscConstants() {}

    public static final int[] USED_CONTROLLER_PORTS = {0, 1};
    public static final boolean TUNING_MODE = true;

    public static final int CONFIGURATION_ATTEMPTS = 10;
  }

  public static final double DT = 0.02;
}
