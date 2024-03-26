package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.SwerveConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.MiscConstants;
import frc.robot.Robot;
import frc.robot.telemetry.types.*;
import frc.robot.telemetry.wrappers.TelemetryPigeon2;
import frc.robot.utils.Alert;
import frc.robot.utils.ConfigurationUtils;
import frc.robot.utils.ConfigurationUtils.StringFaultRecorder;
import java.util.List;
import java.util.function.Function;
import org.photonvision.EstimatedRobotPose;

/** The subsystem containing all the swerve modules */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[NUM_MODULES];

  private final Function<Pose2d, List<EstimatedRobotPose>> cameraPoseDataSupplier;

  private final TelemetryPigeon2 pigeon =
      new TelemetryPigeon2(
          PIGEON_ID, "/drive/pigeon", MiscConstants.CANIVORE_NAME, MiscConstants.TUNING_MODE);

  private StatusSignal<Double> yawSignal;
    private StatusSignal<Double> yawVelSignal;


  private final SwerveDrivePoseEstimator poseEstimator;

  private final SysIdRoutine driveVelocitySysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> voltage) -> setCharacterizationVoltage(voltage.in(Volts)),
              null,
              this,
              "SwerveDrive"));

  private final SysIdRoutine steerPositionSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> voltage) -> setRawVolts(0.0, voltage.in(Volts)),
              null,
              this,
              "SwerveSteer"));

  private final Alert pigeonConfigurationAlert =
      new Alert("Pigeon failed to initialize", Alert.AlertType.ERROR);
  private final DoubleTelemetryEntry gyroEntry =
      new DoubleTelemetryEntry("/drive/gyroRadians", true);

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

  public SwerveDriveSubsystem(Function<Pose2d, List<EstimatedRobotPose>> cameraPoseDataSupplier) {
    modules[0] = new SwerveModule(FRONT_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    modules[1] = new SwerveModule(FRONT_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    modules[2] = new SwerveModule(BACK_LEFT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    modules[3] = new SwerveModule(BACK_RIGHT_MODULE_CONFIGURATION, MiscConstants.TUNING_MODE);
    driveEventLogger.append("Swerve modules initialized");
    configurePigeon();

    this.cameraPoseDataSupplier = cameraPoseDataSupplier;

    poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics,
            getGyroRotation(),
            getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.01),
            VecBuilder.fill(0.9, 0.9, 1));

    // Start odometry thread
    Robot.getInstance().addPeriodic(this::updateOdometry, 1.0 / ODOMETRY_FREQUENCY);

    stopMovement();
  }

  private void configurePigeon() {
    StringFaultRecorder faultRecorder = new StringFaultRecorder();
    yawSignal = pigeon.getYaw();
    yawVelSignal = pigeon.getAngularVelocityZWorld();
    ConfigurationUtils.applyCheckRecordCTRE(
        () -> yawSignal.setUpdateFrequency(ODOMETRY_FREQUENCY),
        () -> yawSignal.getAppliedUpdateFrequency() == ODOMETRY_FREQUENCY,
        faultRecorder.run("Update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
        ConfigurationUtils.applyCheckRecordCTRE(
        () -> yawVelSignal.setUpdateFrequency(ODOMETRY_FREQUENCY),
        () -> yawVelSignal.getAppliedUpdateFrequency() == ODOMETRY_FREQUENCY,
        faultRecorder.run("Vel Update frequency"),
        MiscConstants.CONFIGURATION_ATTEMPTS);
    ConfigurationUtils.applyCheckRecordCTRE(
        pigeon::optimizeBusUtilization,
        () -> true,
        faultRecorder.run("Optimize bus utilization"),
        MiscConstants.CONFIGURATION_ATTEMPTS);

    ConfigurationUtils.postDeviceConfig(
        faultRecorder.hasFault(),
        driveEventLogger::append,
        "Drive Pigeon",
        faultRecorder.getFaultString());
    pigeonConfigurationAlert.set(faultRecorder.hasFault());
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
    return Rotation2d.fromDegrees(StatusSignal.getLatencyCompensatedValue(yawSignal, yawVelSignal));
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
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, Constants.DT)));
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

    desiredSwerveStatesEntry.append(desiredStates);

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_SECOND);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i], activeSteer, openLoop);
    }
  }

  /**
   * Set the voltage directly for the motors.
   *
   * @param driveVolts the desired drive voltage
   * @param steerVolts the desired steer voltage
   */
  public void setRawVolts(double driveVolts, double steerVolts) {
    for (SwerveModule module : modules) {
      module.setRawVoltage(driveVolts, steerVolts);
    }
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
  private void setCharacterizationVoltage(double voltage) {
    for (SwerveModule module : modules) {
      module.setDriveCharacterizationVoltage(voltage);
    }
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

  public double[] getWheelRadiusCharPosition() {
    double[] actualPositions = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      actualPositions[i] = modules[i].getPositionRad();
    }
    return actualPositions;
  }

  public void runWheelCharacterization(double omegaSpeed) {
    setChassisSpeeds(new ChassisSpeeds(0, 0, omegaSpeed), false);
  }

  public Command driveQuasistaticSysIDCommand(SysIdRoutine.Direction direction) {
    return driveVelocitySysId
        .quasistatic(direction)
        .beforeStarting(SignalLogger::start)
        .beforeStarting(Commands.waitSeconds(1.5))
        .beforeStarting(
            () ->
                setRawStates(
                    true,
                    true,
                    new SwerveModuleState[] {
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
                    }));
  }

  public Command driveDynamicSysIDCommand(SysIdRoutine.Direction direction) {
    return driveVelocitySysId
        .dynamic(direction)
        .beforeStarting(SignalLogger::start)
        .beforeStarting(Commands.waitSeconds(1.5))
        .beforeStarting(
            () ->
                setRawStates(
                    true,
                    true,
                    new SwerveModuleState[] {
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
                    }));
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(yawSignal.refresh().getValue());
  }

  public Command steerQuasistaticSysIDCommand(SysIdRoutine.Direction direction) {
    return steerPositionSysId.quasistatic(direction).beforeStarting(SignalLogger::start);
  }

  public Command steerDynamicSysIDCommand(SysIdRoutine.Direction direction) {
    return steerPositionSysId.dynamic(direction).beforeStarting(SignalLogger::start);
  }

  @Override
  public void periodic() {
    List<EstimatedRobotPose> estimatedRobotPoses = cameraPoseDataSupplier.apply(getPose());
    for (EstimatedRobotPose estimatedRobotPose : estimatedRobotPoses) {
      if (!DriverStation.isAutonomousEnabled()) {
        poseEstimator.addVisionMeasurement(
            estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
      }
    }

    logValues();
  }

  private void logValues() {
    gyroEntry.append(getGyroRotation().getRadians());
    chassisSpeedsEntry.append(getCurrentChassisSpeeds());
    actualSwerveStatesEntry.append(getActualStates());
    pigeon.logValues();

    for (SwerveModule module : modules) {
      module.logValues();
    }
    field2d.setRobotPose(getPose());
  }

  public Field2d getField2d() {
    return field2d;
  }
}
