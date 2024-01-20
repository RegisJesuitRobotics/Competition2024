package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.telemetry.tunable.TunableTelemetryProfiledPIDController;
import frc.robot.telemetry.wrappers.TelemetryCANSparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.telemetry.wrappers.TelemetryPigeon2;
import frc.robot.utils.RaiderUtils;

import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {
private final TelemetryCANSparkMax leftMotor = new TelemetryCANSparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless, "elevator/left", true);
private final TelemetryCANSparkMax rightMotor = new TelemetryCANSparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless,"elevator/right",  true);

private final TunableTelemetryProfiledPIDController controller = new TunableTelemetryProfiledPIDController("elevator/controller",PID_GAINS, TRAPEZOIDAL_PROFILE_GAINS);
private SimpleMotorFeedforward feedforward = FF_GAINS.createFeedforward();
private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

private final DigitalInput bottomLimit = new DigitalInput(ELEVATOR_LIMIT_SWITCH);




public ElevatorSubsystem() {

}

public void atBottomLimit(){
    if (bottomLimit.get()){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
}

public void setDesiredPosition(double desiredPosition){
    //TODO: CLAMP THIS
    controller.setGoal(desiredPosition);
}

public boolean atGoal(){
    return controller.atGoal();
}

public void setEncoderPosition(double position){
    leftEncoder.setPosition(position);
    rightEncoder.setPosition(position);
}

public void setVoltage(double voltage){
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
}

public double getPosition(){
    return leftEncoder.getPosition();
}

public void stopMove(){
    rightMotor.setVoltage(0);
    leftMotor.setVoltage(0);
}

@Override
    public void periodic(){
    double feedbackOutput = controller.calculate(getPosition());

    TrapezoidProfile.State currentSetpoint = controller.getSetpoint();
    double combinedOutput = feedbackOutput + feedforward.calculate(currentSetpoint.velocity);
    leftMotor.setVoltage(combinedOutput);
    rightMotor.setVoltage(combinedOutput);

    Robot.startWNode("LogValues");
    logValues();
    Robot.endWNode();
    Robot.endWNode();

}
private void logValues() {
        leftMotor.logValues();
        rightMotor.logValues();
        if (FF_GAINS.hasChanged()) {
            feedforward = FF_GAINS.createFeedforward();
        }
    }




}
