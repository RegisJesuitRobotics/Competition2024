package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RaiderMathUtils {
  private RaiderMathUtils() {}

  public static boolean isChassisSpeedsZero(
      ChassisSpeeds chassisSpeeds, double allowedTranslation, double allowedOmega) {
    return Math.abs(chassisSpeeds.omegaRadiansPerSecond) < allowedOmega
        && Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            < allowedTranslation;
  }

  /**
   * @param value the value
   * @param pow the power to put the value to
   * @return the value returned by with sign copied
   */
  public static double signCopyPow(double value, double pow) {
    return Math.signum(value) * Math.pow(Math.abs(value), pow);
  }

  public static double deadZoneAndCubeJoystick(double value) {
    return signCopyPow(MathUtil.applyDeadband(value, 0.03), 3);
  }
}
