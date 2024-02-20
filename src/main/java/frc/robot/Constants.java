package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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
    public static final double AMP_WRIST_ANGLE = Units.degreesToRadians(110.0);
  }

  public static class IntakeConstants {
    public static final int INTAKE_VOLTAGE = 6;
    public static final int INTAKE_MOTOR_ID = 6;

    public static final int STALL_MOTOR_CURRENT = 40;
    public static final int FREE_MOTOR_CURRENT = 20;
  }

  public static class SlapdownConstants {

    public static final int FEEDER_MOTOR_ID = 7;
    public static final int ROTATION_MOTOR_ID = 8;
    public static final double ROTATION_GEAR_RATIO = 5.0 * 3.0 * 30.0 / 15.0;

    public static final int FEED_STALL_MOTOR_CURRENT = 20;
    public static final int FEED_FREE_MOTOR_CURRENT = 10;

    public static final int ROTATION_STALL_MOTOR_CURRENT = 20;
    public static final int ROTATION_FREE_MOTOR_CURRENT = 10;

    public static final TunablePIDGains ROTATION_GAINS =
        new TunablePIDGains("/Slapdown/rotation/gains", 0, 0, 0, true);
    public static final TunableTrapezoidalProfileGains ROTATION_TRAP_GAINS =
        new TunableTrapezoidalProfileGains("/Slapdown/rotation/trapGains", 0, 0, true);
    public static final TunableFFGains ROTATION_FF_GAINS =
        new TunableFFGains("/Slapdown/rotation/FFGains", 0, 0, 0, true);
  }

  public static class ElevatorConstants {
    public static final boolean ELEVATOR_INVERTED = false;

    public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(11.5);
    public static final double ELEVATOR_MIN_HEIGHT = Units.inchesToMeters(0.0);

    // TODO: Figure these out
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;

    public static final int ELEVATOR_MOTOR_ID = 4;

    public static final int ELEVATOR_LIMIT_SWITCH = 2;

    public static final double ELEVATOR_GEAR_RATIO = 5.0 * 3.0;
    public static final double METERS_PER_REV =
        Units.inchesToMeters((Math.PI * 1.75) / (ELEVATOR_GEAR_RATIO));

    // TODO: Do These PID GAINS
    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("gains/elevator", 57, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
        new TunableTrapezoidalProfileGains("/gains/extension", Units.inchesToMeters(15), Units.inchesToMeters(40), MiscConstants.TUNING_MODE);

    // TODO: TUNE FF GAINS
    public static final TunableArmElevatorFFGains FF_GAINS =
        new TunableArmElevatorFFGains("gains/elevator", 0.22339, 0.041616, 13.1, 1.2556,  MiscConstants.TUNING_MODE);
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

    public static final TunablePIDGains DRIVE_VELOCITY_PID_GAINS =
        new TunablePIDGains("/gains/drive", 0.3, 0.0, 0.0, MiscConstants.TUNING_MODE);

    public static final TunableFFGains DRIVE_VELOCITY_FF_GAINS =
        new TunableFFGains("/gains/drive", 0.2776, 2.32302894, 0.31227, MiscConstants.TUNING_MODE);

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

    public static final double MAX_VELOCITY_METERS_SECOND = 4.2;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_SECOND = Math.PI * 3;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED =
        MAX_ANGULAR_VELOCITY_RADIANS_SECOND / 2.0;

    public static final int ODOMETRY_FREQUENCY = 250;

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
            12, 9, 17, false, true, -0.7455146908760071, false, SHARED_SWERVE_MODULE_CONFIGURATION);
    public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            13, 5, 18, false, true, 2.7857091108003242, false, SHARED_SWERVE_MODULE_CONFIGURATION);

    public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            14, 10, 19, false, true, 2.676796474860444, false, SHARED_SWERVE_MODULE_CONFIGURATION);

    public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            15, 3, 20, false, true, -2.4605051837685683, false, SHARED_SWERVE_MODULE_CONFIGURATION);
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
    public static final Rotation2d WRIST_MIN = new Rotation2d(0);

    public static final int WRIST_MOTOR_ID = 2;
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;

    // TODO: TUNE PID & TRAP & FF
    public static final TunableArmElevatorFFGains WRIST_FF_GAINS =
        new TunableArmElevatorFFGains("/wrist/ffGains", 0, 0, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunablePIDGains WRIST_PID_GAINS =
        new TunablePIDGains("/wrist/pidGains", 0, 0, 0, MiscConstants.TUNING_MODE);

    public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
        new TunableTrapezoidalProfileGains("/wrist/trapGains", 0, 0, MiscConstants.TUNING_MODE);
  }

  public static class TransportConstants {
    public static final int TRANSPORT_MOTOR_ID = 1;
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;
    public static final double TRANSPORT_VOLTAGE = 2.0;
    public static final int SHOOTER_SENSOR_ID = 8;
  }

  public static class ShooterConstants {
    public static final int FREE_MOTOR_CURRENT = 25;
    public static final int STALL_MOTOR_CURRENT = 45;

    public static final int SHOOTER_ID = 11;

    public static final double SHOOTING_RPM = 3000.0;

    public static final Rotation2d SHOOTING_ANGLE =
        new Rotation2d(Units.degreesToRadians(10)); // TODO: Implement Ollies stuff

    public static final double SHOOTER_GEAR_RATIO = 1.0 / 2.0;

    // TODO TUNE FF GAINS
    public static TunablePIDGains SHOOTER_PID_GAINS =
        new TunablePIDGains("/shooter/pid", 0.024885, 0, 0, MiscConstants.TUNING_MODE);
    public static TunableFFGains SHOOTER_FF_GAINS =
        new TunableFFGains("/shooter/FF", 0.34964, 0.05597, 0.01424, MiscConstants.TUNING_MODE);
  }

  public static class TeleopConstants {
    private TeleopConstants() {}

    public static final boolean OPEN_LOOP_DRIVETRAIN = true;
    public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED = 10.0;
    public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED = 5.0 * Math.PI;
    public static final double MINIMUM_VELOCITY_METERS_SECOND = 0.10;
    public static final double MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND = 0.10;
  }

  public static class VisionConstants {
    private VisionConstants() {}

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
  }

  public static class MiscConstants {

    private MiscConstants() {}

    public static final int[] USED_CONTROLLER_PORTS = {0, 1};
    public static final boolean TUNING_MODE = true;

    public static final ModuleType POWER_MODULE_TYPE = ModuleType.kRev;
    public static final int POWER_MODULE_ID = 0;
    public static final int CONFIGURATION_ATTEMPTS = 5;
  }

  public static final double DT = 0.02;
}
