package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveConstants.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.types.*;
import frc.robot.telemetry.wrappers.TelemetryPigeon2;
import frc.robot.utils.RaiderMathUtils;
import frc.robot.utils.RaiderUtils;

/** The subsystem containing all the swerve modules */
public class SwerveDriveSubsystem extends SubsystemBase {

  enum DriveMode {
    OPEN_LOOP,
    CLOSE_LOOP,
    CHARACTERIZATION,
    RAW_VOLTAGE
  }

  private final SwerveModule[] modules = new SwerveModule[NUM_MODULES];

  private final TelemetryPigeon2 pigeon =
      new TelemetryPigeon2(GYRO_ID, "/drive/gyro", MiscConstants.TUNING_MODE);
  private final StatusSignal<Double> yawSignal = pigeon.getYaw();

  private final SwerveDrivePoseEstimator poseEstimator;

  private final BooleanTelemetryEntry allModulesAtAbsoluteZeroEntry =
      new BooleanTelemetryEntry("/drive/allModulesAtAbsoluteZero", true);
  private final DoubleTelemetryEntry gyroEntry =
      new DoubleTelemetryEntry("/drive/gyroDegrees", true);
  private final StructTelemetryEntry<ChassisSpeeds> chassisSpeedsEntry =
      new StructTelemetryEntry<>("/drive/speeds", ChassisSpeeds.struct, MiscConstants.TUNING_MODE);
  private final StructTelemetryEntry<ChassisSpeeds> desiredSpeedsEntry =
      new StructTelemetryEntry<>(
          "/drive/desiredSpeeds", ChassisSpeeds.struct, MiscConstants.TUNING_MODE);
  private final StructTelemetryEntry<Pose2d> odometryEntry =
      new StructTelemetryEntry<>("/drive/estimatedPose", Pose2d.struct, MiscConstants.TUNING_MODE);
  private final StructArrayTelemetryEntry<SwerveModuleState> desiredSwerveStatesEntry =
      new StructArrayTelemetryEntry<>(
          "/drive/desiredStates", SwerveModuleState.struct, MiscConstants.TUNING_MODE);
  private final StructArrayTelemetryEntry<SwerveModuleState> actualSwerveStatesEntry =
      new StructArrayTelemetryEntry<>(
          "/drive/actualStates", SwerveModuleState.struct, MiscConstants.TUNING_MODE);
  private final EventTelemetryEntry driveEventLogger = new EventTelemetryEntry("/drive/events");

  private final Field2d field2d = new Field2d();

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  private SwerveModuleState[] desiredStates = new SwerveModuleState[NUM_MODULES];
  private boolean activeSteer = true;
  private DriveMode driveMode = DriveMode.OPEN_LOOP;
  private double rawDriveVolts = 0.0;
  private double rawSteerVolts = 0.0;

  public SwerveDriveSubsystem() {
    modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);

    driveEventLogger.append("Swerve modules initialized");

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, getGyroRotation(), getModulePositions(), new Pose2d());

    RaiderUtils.applyAndCheckCTRE(
        () -> yawSignal.setUpdateFrequency(ODOMETRY_FREQUENCY),
        () -> yawSignal.getAppliedUpdateFrequency() == ODOMETRY_FREQUENCY,
        MiscConstants.CONFIGURATION_ATTEMPTS);
    RaiderUtils.applyAndCheckCTRE(
        pigeon::optimizeBusUtilization, () -> true, MiscConstants.CONFIGURATION_ATTEMPTS);

    // Start odometry thread
    Robot.getInstance().addPeriodic(this::updateOdometry, 1.0 / ODOMETRY_FREQUENCY);

    stopMovement();
  }

  private void updateOdometry() {
    Pose2d pose = poseEstimator.update(getGyroRotation(), getModulePositions());
    odometryEntry.append(pose);
  }

  /**
   * @return the value from the gyro. This does not get reset when resetOdometry is called. Use
   *     <code>getPose().getRotation2d()</code> for reset value. Counterclockwise is positive.
   */
  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(yawSignal.refresh().getValue());
  }

  /** Sets the odometry perceived location to zero */
  public void zeroHeading() {
    setHeading(Rotation2d.fromDegrees(0.0));
  }

  /**
   * Set the odometry perceived location to the provided heading
   *
   * @param newHeading the provided heading
   */
  public void setHeading(Rotation2d newHeading) {
    Pose2d currentPose = getPose();

    Pose2d newPose = new Pose2d(currentPose.getTranslation(), newHeading);

    resetOdometry(newPose);
  }

  /**
   * Set the current perceived location of the robot to the provided pose
   *
   * @param pose2d the provided pose
   */
  public void resetOdometry(Pose2d pose2d) {
    poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose2d);

    driveEventLogger.append("Estimator reset");
  }

  /**
   * @return the estimated position of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return kinematics.toChassisSpeeds(getActualStates());
  }

  /**
   * Set the desired speed of the robot. Chassis speeds are always robot centric but can be created
   * from field centric values through {@link ChassisSpeeds#fromFieldRelativeSpeeds(double, double,
   * double, Rotation2d)}
   *
   * @param chassisSpeeds the desired chassis speeds
   * @param openLoop if true then velocity will be handled exclusivity with feedforward (mostly used
   *     for teleop). If false a PIDF will be used (mostly used for auto)
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean openLoop) {
    desiredSpeedsEntry.append(chassisSpeeds);

    setRawStates(
        true,
        openLoop,
        kinematics.toSwerveModuleStates(RaiderMathUtils.correctForSwerveSkew(chassisSpeeds)));
  }

  /**
   * Sets the desired swerve drive states for the modules. This method also takes a copy of the
   * states, so they will not be changed. Assumes zero acceleration.
   *
   * @param activeSteer if false will not actively power the steer motor
   * @param openLoop if true then velocity will be handled exclusivity with feedforward (for teleop
   *     mostly). If false a PIDF will be used (for auto)
   * @param desiredStates the desired states... Ordered front left, front right, back left, back
   *     right
   */
  public void setRawStates(
      boolean activeSteer, boolean openLoop, SwerveModuleState[] desiredStates) {
    if (desiredStates.length != modules.length) {
      throw new IllegalArgumentException("You must provide desiredStates for all modules");
    }

    driveMode = openLoop ? DriveMode.OPEN_LOOP : DriveMode.CLOSE_LOOP;
    this.activeSteer = activeSteer;

    this.desiredStates = RaiderMathUtils.copySwerveStateArray(desiredStates);
  }

  /**
   * Set the voltage directly for the motors.
   *
   * @param driveVolts the desired drive voltage
   * @param steerVolts the desired steer voltage
   */
  public void setRawVolts(double driveVolts, double steerVolts) {
    driveMode = DriveMode.RAW_VOLTAGE;

    this.rawDriveVolts = driveVolts;
    this.rawSteerVolts = steerVolts;
  }

  /** Sets each module velocity to zero and desired angle to what it currently is */
  public void stopMovement() {
    SwerveModuleState[] newStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      newStates[i] = new SwerveModuleState(0.0, modules[i].getActualState().angle);
    }
    setRawStates(false, true, newStates);
  }

  /**
   * Set the module to characterization mode with the provided voltage
   *
   * @param voltage the voltage to apply to the drive motor
   */
  public void setCharacterizationVoltage(double voltage) {
    driveMode = DriveMode.CHARACTERIZATION;
    rawDriveVolts = voltage;
  }

  public double[] getActualDriveVoltages() {
    double[] voltages = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      voltages[i] = modules[i].getActualDriveVoltage();
    }
    return voltages;
  }

  public void setAllModulesToAbsolute() {
    for (SwerveModule module : modules) {
      module.resetSteerToAbsolute();
    }
  }

  private boolean allModulesAtAbsolute() {
    boolean allSet = true;
    for (SwerveModule module : modules) {
      allSet &= module.isSetToAbsolute();
    }
    return allSet;
  }

  /**
   * Should only be used for characterization
   *
   * @return the angle in radians
   */
  public double getRawGyroAngle() {
    return Units.degreesToRadians(pigeon.getAngle());
  }

  /**
   * Should only be used for characterization
   *
   * @return the angle rate in radians/second
   */
  public double getRawGyroRate() {
    return Units.degreesToRadians(pigeon.getRate());
  }

  public SwerveModuleState[] getActualStates() {
    SwerveModuleState[] actualStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      actualStates[i] = modules[i].getActualState();
    }
    return actualStates;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] actualPositions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      actualPositions[i] = modules[i].getActualPosition();
    }

    return actualPositions;
  }

  @Override
  public void periodic() {
    switch (driveMode) {
      case OPEN_LOOP, CLOSE_LOOP -> {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_SECOND);
        for (int i = 0; i < modules.length; i++) {
          modules[i].setDesiredState(
              desiredStates[i], activeSteer, driveMode == DriveMode.OPEN_LOOP);
        }
      }
      case RAW_VOLTAGE -> {
        for (SwerveModule module : modules) {
          module.setRawVoltage(rawDriveVolts, rawSteerVolts);
        }
      }
      case CHARACTERIZATION -> {
        for (SwerveModule module : modules) {
          module.setCharacterizationVoltage(rawDriveVolts);
        }
      }
    }

    logValues();
  }

  private void logValues() {
    allModulesAtAbsoluteZeroEntry.append(allModulesAtAbsolute());
    gyroEntry.append(getGyroRotation().getDegrees());
    chassisSpeedsEntry.append(getCurrentChassisSpeeds());
    desiredSwerveStatesEntry.append(desiredStates);
    actualSwerveStatesEntry.append(getActualStates());

    for (SwerveModule module : modules) {
      module.logValues();
    }
    field2d.setRobotPose(getPose());
  }

  public Field2d getField2d() {
    return field2d;
  }
}
