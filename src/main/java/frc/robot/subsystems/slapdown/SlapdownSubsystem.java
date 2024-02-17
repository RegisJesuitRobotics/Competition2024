package frc.robot.subsystems.slapdown;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryCANSparkFlex;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.utils.Alert;
import frc.robot.utils.RaiderUtils;

import static frc.robot.Constants.SlapdownConstants.*;

public class SlapdownSubsystem extends SubsystemBase {

    private Alert rotationMotorAlert;
    private final TelemetryCANSparkMax feederMotor = new TelemetryCANSparkMax(FEEDER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/Slapdown/feeder/motor", false);
    private final TelemetryCANSparkFlex rotationMotor = new TelemetryCANSparkFlex(ROTATION_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless, "/Slapdown/rotation/motor", true);

    private SimpleMotorFeedforward rotationFF;
    private TunableTelemetryProfiledPIDController rotationController = new TunableTelemetryProfiledPIDController("/Slapdown/rotation/controller", ROTATION_GAINS, ROTATION_TRAP_GAINS);
    private final RelativeEncoder rotationEncoder;
    public SlapdownSubsystem(){
        rotationEncoder = rotationMotor.getEncoder();
        rotationMotorAlert = new Alert("Rotation Slapdown Motor: ", Alert.AlertType.ERROR);
        rotationFF = ROTATION_FF_GAINS.createFeedforward();

        configMotors();
    }

    private void configMotors(){
        boolean faultInitializing = false;

        double rotationConversionFactor = (Math.PI) / ROTATION_GEAR_RATIO;

        faultInitializing |=
                RaiderUtils.applyAndCheckRev(
                        () -> rotationMotor.setCANTimeout(250),
                        () -> true,
                        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

        faultInitializing |=
                RaiderUtils.applyAndCheckRev(
                        rotationMotor::restoreFactoryDefaults,
                        () -> true,
                        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);
        faultInitializing |=
                RaiderUtils.applyAndCheckRev(
                        () -> rotationMotor.setSmartCurrentLimit(STALL_MOTOR_CURRENT, FREE_MOTOR_CURRENT),
                        () -> true,
                        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

        faultInitializing |=
                RaiderUtils.applyAndCheckRev(
                        () -> rotationMotor.setIdleMode(CANSparkMax.IdleMode.kCoast),
                        () -> rotationMotor.getIdleMode() == CANSparkMax.IdleMode.kCoast,
                        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

        faultInitializing |=
                RaiderUtils.applyAndCheckRev(
                        rotationMotor::burnFlashWithDelay,
                        () -> true,
                        Constants.MiscConstants.CONFIGURATION_ATTEMPTS);

        rotationMotorAlert.set(faultInitializing);
    }

    public void setFeederVoltage(double voltage){
        feederMotor.setVoltage(voltage);
    }
    private void setRotationGoal(Rotation2d goal){
        rotationController.setGoal(goal.getRadians());
    }

    public void setRotationVoltage(double voltage){
        rotationMotor.setVoltage(voltage);
    }
    private double getPosition(){
        return rotationEncoder.getPosition();
    }

    public void setSlapdownDown(){

    }

    public Command setRotationGoalCommand(Rotation2d goal){
        return this.run(
                () -> this.setRotationGoal(goal));

    }

    public Command setFeederVoltageCommand(double voltage){
        return this.run(
                () -> this.setFeederVoltage(voltage)
        );
    }

    @Override
    public void periodic(){
        double feedbackOutput = rotationController.calculate(getPosition());

        TrapezoidProfile.State currentSetpoint = rotationController.getSetpoint();
        double combinedOutput = feedbackOutput + rotationFF.calculate(currentSetpoint.velocity);
        setRotationVoltage(combinedOutput);
    }


}
