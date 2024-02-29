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
      double driveCurrentLimit,
      double steerCurrentLimit,
      double wheelDiameterMeters,
      double freeSpeedMetersPerSecond,
      int odometryFrequency,
      TunablePIDGains driveVelocityPIDGains,
      TunableFFGains driveVelocityFFGains,
      TunablePIDGains steerPositionPIDGains,
      TunableFFGains steerVelocityFFGains) {}
}
