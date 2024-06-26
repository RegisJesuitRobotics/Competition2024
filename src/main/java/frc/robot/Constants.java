package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.telemetry.tunable.gains.TunableArmElevatorFFGains;
import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;
import frc.robot.telemetry.tunable.gains.TunableTrapezoidalProfileGains;
import frc.robot.utils.SwerveModuleConfiguration;
import frc.robot.utils.SwerveModuleConfiguration.SharedSwerveModuleConfiguration;
import java.util.Collections;
import java.util.List;

/** File containing all constants for the robot. */
public final class Constants {
  private Constants() {}

  public static class SetpointConstants {

    public static final double REGULAR_SHOT_ELEVATOR_HEIGHT_METERS = Units.inchesToMeters(0.0);
    public static final InterpolatingDoubleTreeMap REGULAR_SHOT_WRIST_SETPOINT_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(0.0, WristConstants.WRIST_MIN_RADIANS);
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(
          1.24, Units.degreesToRadians(42.0) - WristConstants.WRIST_TO_SHOOTER);
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(
          1.54, Units.degreesToRadians(44.0) - WristConstants.WRIST_TO_SHOOTER);
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(
          1.95, Units.degreesToRadians(50.0) - WristConstants.WRIST_TO_SHOOTER);
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(
          2.43, Units.degreesToRadians(58.0) - WristConstants.WRIST_TO_SHOOTER);
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(
          2.89, Units.degreesToRadians(62.0) - WristConstants.WRIST_TO_SHOOTER);
      REGULAR_SHOT_WRIST_SETPOINT_TABLE.put(
          3.48, Units.degreesToRadians(66.0) - WristConstants.WRIST_TO_SHOOTER);
    }

    public static final double HIGH_SHOT_ELEVATOR_HEIGHT_METERS = Units.inchesToMeters(10.0);
    public static final InterpolatingDoubleTreeMap HIGH_SHOT_WRIST_SETPOINT_TABLE =
        new InterpolatingDoubleTreeMap();

    static {
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(0.0, WristConstants.WRIST_MIN_RADIANS);
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(1.1, WristConstants.WRIST_MIN_RADIANS);
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(
          1.6, Units.degreesToRadians(28) - WristConstants.WRIST_TO_SHOOTER);
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(
          2.1, Units.degreesToRadians(34.0) - WristConstants.WRIST_TO_SHOOTER);
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(
          2.6, Units.degreesToRadians(34.67) - WristConstants.WRIST_TO_SHOOTER);
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(
          2.8, Units.degreesToRadians(32.0) - WristConstants.WRIST_TO_SHOOTER);
      HIGH_SHOT_WRIST_SETPOINT_TABLE.put(
          3.1, Units.degreesToRadians(32.0) - WristConstants.WRIST_TO_SHOOTER);
    }

    public static final double SHOOT_SHOOTER_VELOCITY =
        Units.rotationsPerMinuteToRadiansPerSecond(6000.0);
    public static final double AMP_SHOOTER_VELOCITY =
        Units.rotationsPerMinuteToRadiansPerSecond(2000.0);
    public static final double IDLE_SHOOTER_VELOCITY =
        Units.rotationsPerMinuteToRadiansPerSecond(1500.0);

    public static final double AMP_ELEVATOR_HEIGHT = Units.inchesToMeters(3.0);
    public static final double AMP_WRIST_ANGLE_RADIANS =
        Units.degreesToRadians(130) - WristConstants.WRIST_TO_SHOOTER;

    public static final double INTAKE_ELEVATOR_HEIGHT = Units.inchesToMeters(0.0);
    public static final double INTAKE_WRIST_ANGLE_RADIANS = WristConstants.WRIST_MIN_RADIANS;

    public static final double CLOSE_SPEAKER_WRIST_ANGLE_RADIANS =
        Units.degreesToRadians(42) - WristConstants.WRIST_TO_SHOOTER;
    public static final double EXPEL_ELEVATOR_HEIGHT = Units.inchesToMeters(0.0);
    public static final double EXPEL_WRIST_ANGLE_RADIANS =
        Units.degreesToRadians(90) - WristConstants.WRIST_TO_SHOOTER;

    // 11
    public static final double CLIMB_UP_ELEVATOR_HEIGHT = Units.inchesToMeters(10.0);
    public static final double CLIMB_UP_WRIST_ANGLE_RADIANS =
        Units.degreesToRadians(140) - WristConstants.WRIST_TO_SHOOTER;
    public static final double CLIMB_DOWN_WRIST_ANGLE_RADIANS =
        Units.degreesToRadians(140) - WristConstants.WRIST_TO_SHOOTER;
  }

  public static class IntakeConstants {
    public static final double INTAKE_VOLTAGE = 9;
    public static final int INTAKE_MOTOR_ID = 6;
    public static final boolean INVERTED = false;

    public static final int STALL_MOTOR_CURRENT = 40;
    public static final int FREE_MOTOR_CURRENT = 30;
  }

  public static class SlapdownConstants {
    public static final int FEEDER_MOTOR_ID = 7;
    public static final int ROTATION_MOTOR_ID = 9;

    public static final boolean FEEDER_INVERTED = true;
    public static final boolean ROTATION_INVERTED = false;

    public static final double ROTATION_GEAR_RATIO = 5.0 * 3.0 * 30.0 / 15.0;
    public static final double FEEDER_GEAR_RATIO = 5.0;

    public static final int FEED_STALL_MOTOR_CURRENT = 20;
    public static final int FEED_FREE_MOTOR_CURRENT = 20;

    public static final int ROTATION_STALL_MOTOR_CURRENT = 25;
    public static final int ROTATION_FREE_MOTOR_CURRENT = 20;

    public static final double ROTATION_UP_ANGLE = 1.9347;
    public static final double ROTATION_DOWN_ANGLE = 4.25;

    public static final int ROTATION_LIMIT_SWITCH_ID = 5;

    public static final double FEEDER_VOLTAGE = 9;

    public static final TunablePIDGains ROTATION_GAINS =
        new TunablePIDGains("/gains/slapdownRotation", 5, 0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains ROTATION_TRAP_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/slapdownRotation", 25, 30, MiscConstants.TUNING_MODE);
    public static final TunableArmElevatorFFGains ROTATION_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/gains/slapdownRotation",
            0.28058,
            0.085329,
            0.5824,
            0.081934,
            MiscConstants.TUNING_MODE);
  }

  public static class ElevatorConstants {
    public static final boolean MAIN_INVERTED = false;
    public static final boolean FOLLOWER_INVERTED = false;

    public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(11.0);
    public static final double ELEVATOR_MIN_HEIGHT = Units.inchesToMeters(0.0);

    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 40;

    public static final int ELEVATOR_MOTOR_ID = 4;
    public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 22;

    public static final int ELEVATOR_LIMIT_SWITCH = 6;

    public static final double ELEVATOR_GEAR_RATIO = 4.0 * 4.0 * 3.0;
    public static final double METERS_PER_REV =
        (Math.PI * Units.inchesToMeters(1.75)) / (ELEVATOR_GEAR_RATIO);

    public static final TunablePIDGains PID_GAINS =
        new TunablePIDGains("/gains/elevator", 42.0, 0.0, 0.0, true);
    public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/elevator", 0.25, 0.25, MiscConstants.TUNING_MODE);

    public static final TunableArmElevatorFFGains FF_GAINS =
        new TunableArmElevatorFFGains(
            "/gains/elevator", 0.15013, 0.053896, 43.417, 4.0148, MiscConstants.TUNING_MODE);
  }

  public static class SwerveConstants {
    private SwerveConstants() {}

    public static final int PIGEON_ID = 21;

    public static final int NUM_MODULES = 4;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(1.9414427803834284 * 2);
    public static final double DRIVE_GEAR_REDUCTION = (50.0 / 16) * (17.0 / 27) * (45.0 / 15);

    public static final double STEER_GEAR_REDUCTION = 150.0 / 7.0;

    public static final double DRIVE_CURRENT_LIMIT = 60.0;
    public static final double STEER_CURRENT_LIMIT = 35.0;

    // 0.47
    public static final TunablePIDGains DRIVE_VELOCITY_PID_GAINS =
        new TunablePIDGains("/gains/drive", 0.26125, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains DRIVE_VELOCITY_FF_GAINS =
        new TunableFFGains("/gains/drive", 0.24872, 0.11031, 0.0080533, MiscConstants.TUNING_MODE);

    public static final TunablePIDGains STEER_POSITION_PID_GAINS =
        new TunablePIDGains("/gains/steer", 110, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableFFGains STEER_VELOCITY_FF_GAINS =
        new TunableFFGains("/gains/steer", 0.27012, 2.2037, 0.059669, MiscConstants.TUNING_MODE);

    // Left right distance between center of wheels
    public static final double TRACKWIDTH_METERS = Units.inchesToMeters(22.75);

    // Front back distance between center of wheels
    public static final double WHEELBASE_METERS = Units.inchesToMeters(21.75);

    public static final double WHEELBASE_RADIUS =
        Math.sqrt(Math.pow(WHEELBASE_METERS, 2) + Math.pow(TRACKWIDTH_METERS, 2));

    public static final Translation2d[] MODULE_TRANSLATIONS =
        new Translation2d[] {
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)
        };

    public static final double MAX_VELOCITY_METERS_SECOND = 4.3;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_SECOND = 10;
    public static final double MAX_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED = 36;

    public static final int ODOMETRY_FREQUENCY = 250;

    private static final SharedSwerveModuleConfiguration SHARED_SWERVE_MODULE_CONFIGURATION =
        new SharedSwerveModuleConfiguration(
            MiscConstants.CANIVORE_NAME,
            DRIVE_GEAR_REDUCTION,
            STEER_GEAR_REDUCTION,
            DRIVE_CURRENT_LIMIT,
            STEER_CURRENT_LIMIT,
            WHEEL_DIAMETER_METERS,
            MAX_VELOCITY_METERS_SECOND,
            ODOMETRY_FREQUENCY,
            DRIVE_VELOCITY_PID_GAINS,
            DRIVE_VELOCITY_FF_GAINS,
            STEER_POSITION_PID_GAINS,
            STEER_VELOCITY_FF_GAINS);

    public static final SwerveModuleConfiguration FRONT_LEFT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            12, 8, 17, true, true, -2.440563, false, SHARED_SWERVE_MODULE_CONFIGURATION);
    public static final SwerveModuleConfiguration FRONT_RIGHT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            13, 5, 18, true, true, -0.348214, false, SHARED_SWERVE_MODULE_CONFIGURATION);
    public static final SwerveModuleConfiguration BACK_LEFT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            14, 10, 19, true, true, -0.480136, false, SHARED_SWERVE_MODULE_CONFIGURATION);
    public static final SwerveModuleConfiguration BACK_RIGHT_MODULE_CONFIGURATION =
        new SwerveModuleConfiguration(
            15, 3, 20, true, true, -0.719437, false, SHARED_SWERVE_MODULE_CONFIGURATION);
  }

  public static class AutoConstants {
    private AutoConstants() {}

    public static final double MAX_AUTO_VELOCITY_METERS_SECOND = 3.8;
    public static final double MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED = 5;

    public static final double MAX_AUTO_ANGULAR_VELOCITY_RADIANS_SECOND = 7.0;
    public static final double MAX_AUTO_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED = 30.0;

    public static final TunablePIDGains TRANSLATION_POSITION_GAINS =
        new TunablePIDGains("/gains/driveXY", 4.1, 0.0, 0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains TRANSLATION_POSITION_TRAPEZOIDAL_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/driveXY",
            MAX_AUTO_VELOCITY_METERS_SECOND,
            MAX_AUTO_ACCELERATION_METERS_PER_SECOND_SQUARED,
            MiscConstants.TUNING_MODE);
    public static final TunablePIDGains ANGULAR_POSITION_PID_GAINS =
        new TunablePIDGains("/gains/driveAngular", 4, 0, 0.0, MiscConstants.TUNING_MODE);

    public static final TunablePIDGains SNAP_POSITION_PID_GAINS =
        new TunablePIDGains("/gains/snap", 5, 0, 0.0, MiscConstants.TUNING_MODE);
    public static final TunableTrapezoidalProfileGains ANGULAR_POSITION_TRAPEZOIDAL_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/driveAngular",
            MAX_AUTO_ANGULAR_VELOCITY_RADIANS_SECOND,
            MAX_AUTO_ANGULAR_ACCELERATION_RADIANS_SECOND_SQUARED,
            MiscConstants.TUNING_MODE);
  }

  public static class WristConstants {
    public static final double WRIST_GEAR_RATIO = 25.0 * 42.0 / 18.0;

    public static final double WRIST_MIN_RADIANS = -0.22169540258784437;

    public static final double WRIST_TO_SHOOTER = Units.degreesToRadians(26) - WRIST_MIN_RADIANS;

    public static final double WRIST_OFFSET = -2.1184;
    public static final int WRIST_ENCODER_PORT = 7;

    public static final int WRIST_MOTOR_ID = 2;
    public static final boolean INVERTED = true;
    public static final int STALL_MOTOR_CURRENT = 45;
    public static final int FREE_MOTOR_CURRENT = 25;

    public static final TunableArmElevatorFFGains WRIST_FF_GAINS =
        new TunableArmElevatorFFGains(
            "/gains/wrist/", 0.09409, 0.2675, 1.0086, 0.084547, MiscConstants.TUNING_MODE);
    public static final TunablePIDGains WRIST_PID_GAINS =
        new TunablePIDGains("/gains/wrist/", 4, 0.0, 0.0, MiscConstants.TUNING_MODE);

    public static final TunableTrapezoidalProfileGains TRAPEZOIDAL_PROFILE_GAINS =
        new TunableTrapezoidalProfileGains(
            "/gains/wrist",
            Units.rotationsToRadians(1.5),
            Units.rotationsToRadians(2),
            MiscConstants.TUNING_MODE);
    public static final double DYNAMIC_OFFSET = Units.degreesToRadians(1);
  }

  public static class TransportConstants {
    public static final int TRANSPORT_MOTOR_ID = 1;
    public static final boolean INVERTED = false;
    public static final int STALL_MOTOR_CURRENT = 30;
    public static final int FREE_MOTOR_CURRENT = 20;
    public static final double TRANSPORT_LOAD_VOLTAGE = 6;

    public static final double GEAR_RATIO = 9.0;

    public static final double TRANSPORT_CLOSE_SPEAKER_VOLTAGE = 8;
    public static final int SHOOTER_SENSOR_ID = 8;
  }

  public static class ShooterConstants {
    public static final int FREE_MOTOR_CURRENT = 25;
    public static final int STALL_MOTOR_CURRENT = 80;

    public static final boolean INVERTED = true;
    public static final boolean INVERTED_FOLLOWER = false;
    public static final int SHOOTER_ID = 11;
    public static final int SHOOTER_FOLLOWER_ID = 23;

    public static final double SHOOTER_GEAR_RATIO = 18.0 / 24.0;

    public static TunablePIDGains SHOOTER_PID_GAINS =
        new TunablePIDGains("/gains/shooter", 2.2552E-25, 0.0, 0.0, MiscConstants.TUNING_MODE);
    public static TunableFFGains SHOOTER_FF_GAINS =
        new TunableFFGains(
            "/gains/shooter", 0.23944, 0.013147, 0.00081173, MiscConstants.TUNING_MODE);
  }

  public static class TeleopConstants {
    private TeleopConstants() {}

    public static final boolean OPEN_LOOP_DRIVETRAIN = true;
    public static final double TRANSLATION_RATE_LIMIT_METERS_SECOND_SQUARED = 10;
    public static final double ANGULAR_RATE_LIMIT_RADIANS_SECOND_SQUARED = 40;
    public static final double MINIMUM_VELOCITY_METERS_SECOND = 0.05;
    public static final double MINIMUM_ANGULAR_VELOCITY_RADIANS_SECOND = 0.1;
  }

  public static class VisionConstants {
    private VisionConstants() {}

    public static final Transform3d ROBOT_TO_CAM =
        new Transform3d(
            Units.inchesToMeters(-8.017),
            Units.inchesToMeters(0),
            Units.inchesToMeters(17.34),
            new Rotation3d(0, -Units.degreesToRadians(27.088), Math.PI));
  }

  public static class MiscConstants {
    public static final String CANIVORE_NAME = "canivore";

    private MiscConstants() {}

    public static final int[] USED_CONTROLLER_PORTS = {0, 1};
    public static final boolean TUNING_MODE = !DriverStation.isFMSAttached();

    public static final int CONFIGURATION_ATTEMPTS = 10;
  }

  public static class LEDConstants {
    public static final int PWM_PORT = 0;
    public static final int FRONT_LEFT_SIZE = 12;
    public static final int FRONT_RIGHT_SIZE = 12;
    public static final int STATUS_DEDICATED_SIZE = 2;
    public static final int BACK_LEFT_SIZE = 14;
    public static final int BACK_RIGHT_SIZE = 14;

    public static final int TOTAL_SIZE =
        FRONT_LEFT_SIZE + FRONT_RIGHT_SIZE + BACK_LEFT_SIZE + BACK_RIGHT_SIZE;
    public static final int MAX_SIZE =
        Collections.max(
            List.of(BACK_LEFT_SIZE, FRONT_LEFT_SIZE, BACK_RIGHT_SIZE, FRONT_RIGHT_SIZE));
  }

  public static final double DT = 0.02;
}
