package frc.robot.utils;

import frc.robot.telemetry.tunable.gains.TunableFFGains;
import frc.robot.telemetry.tunable.gains.TunablePIDGains;

public record SwerveModuleConfiguration(
    int driveMotorID,
    int steerMotorID,
    int steerEncoderID,
    boolean driveMotorInverted,
    boolean steerMotorInverted,
    double steerOffsetRadians,
    boolean steerEncoderInverted,
    SharedSwerveModuleConfiguration sharedConfiguration) {
  /** This is all the options that are not module specific */
  public record SharedSwerveModuleConfiguration(
      String canBus,
      double driveGearRatio,
      double steerGearRatio,
      double drivePeakCurrentLimit,
      double driveContinuousCurrentLimit,
      double drivePeakCurrentDurationSeconds,
      int steerFreeCurrentLimit,
      int steerStallCurrentLimit,
      double wheelDiameterMeters,
      int odometryFrequency,
      TunablePIDGains driveVelocityPIDGains,
      TunableFFGains driveVelocityFFGains,
      TunablePIDGains steerPositionPIDGains) {}
}
