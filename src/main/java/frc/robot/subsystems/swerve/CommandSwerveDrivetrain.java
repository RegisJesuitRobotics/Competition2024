package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.telemetry.types.StructArrayTelemetryEntry;
import frc.robot.telemetry.types.StructTelemetryEntry;
import frc.robot.telemetry.wrappers.TelemetryPigeon2;
import frc.robot.utils.SwerveVoltageRequest;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private static final StructArrayTelemetryEntry<SwerveModuleState> desiredStates =
      new StructArrayTelemetryEntry<>(
          "/drive/desiredStates", SwerveModuleState.struct, Constants.MiscConstants.TUNING_MODE);
  private static final StructArrayTelemetryEntry<SwerveModuleState> actualStates =
      new StructArrayTelemetryEntry<>(
          "/drive/actualStates", SwerveModuleState.struct, Constants.MiscConstants.TUNING_MODE);
  private static final StructTelemetryEntry<Pose2d> odometryEntry =
      new StructTelemetryEntry<>("/drive/pose", Pose2d.struct, Constants.MiscConstants.TUNING_MODE);

  private final TelemetryPigeon2 pigeon =
      new TelemetryPigeon2(
          this.m_pigeon2.getDeviceID(),
          "/drive/pigeon",
          Constants.MiscConstants.CANIVORE_NAME,
          Constants.MiscConstants.TUNING_MODE);

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    desiredStates.append(this.getState().ModuleTargets);
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  private SwerveVoltageRequest driveVoltageRequest = new SwerveVoltageRequest(true);

  private final SysIdRoutine driveVelocitySysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> voltage) ->
                  setControl(driveVoltageRequest.withVoltage(voltage.in(Volts))),
              null,
              this,
              "SwerveDrive"));
  private SwerveVoltageRequest steerVoltageRequest = new SwerveVoltageRequest(true);

  private final SysIdRoutine steerPositionSysId =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, null, null, (state) -> SignalLogger.writeString("State", state.toString())),
          new SysIdRoutine.Mechanism(
              (Measure<Voltage> voltage) ->
                  setControl(steerVoltageRequest.withVoltage(voltage.in(Volts))),
              null,
              this,
              "SwerveSteer"));

  public Command driveQTest(SysIdRoutine.Direction direction) {
    return driveVelocitySysId.quasistatic(direction);
  }

  public Command driveDTest(SysIdRoutine.Direction direction) {
    return driveVelocitySysId.dynamic(direction);
  }

  public Command steerQTest(SysIdRoutine.Direction direction) {
    return steerPositionSysId.quasistatic(direction);
  }

  public Command steerDTest(SysIdRoutine.Direction direction) {
    return steerPositionSysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    logValues();
  }

  private void logValues() {
    actualStates.append(this.getState().ModuleStates);
    odometryEntry.append(this.m_odometry.getEstimatedPosition());
    pigeon.logValues();
  }
}
