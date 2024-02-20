package frc.robot.utils;

import static frc.robot.utils.ConfigurationUtils.fpEqual;

import com.ctre.phoenix6.configs.*;

public class ConfigEquality {
  private ConfigEquality() {}

  public static boolean isCANcoderConfigurationEqual(
      CANcoderConfiguration config1, CANcoderConfiguration config2) {
    return isMagnetSensorEqual(config1.MagnetSensor, config2.MagnetSensor);
  }

  private static boolean isMagnetSensorEqual(MagnetSensorConfigs obj1, MagnetSensorConfigs obj2) {
    return obj1.SensorDirection.value == obj2.SensorDirection.value
        && fpEqual(obj1.MagnetOffset, obj2.MagnetOffset)
        && obj1.AbsoluteSensorRange.value == obj2.AbsoluteSensorRange.value;
  }

  // This class is pain, but necessary because of floating point comparison
  public static boolean isTalonConfigurationEqual(
      TalonFXConfiguration config1, TalonFXConfiguration config2) {
    return isMotorOutputEqual(config1.MotorOutput, config2.MotorOutput)
        && isCurrentLimitEqual(config1.CurrentLimits, config2.CurrentLimits)
        && isVoltageEqual(config1.Voltage, config2.Voltage)
        && isTorqueCurrentEqual(config1.TorqueCurrent, config2.TorqueCurrent)
        && isFeedbackEqual(config1.Feedback, config2.Feedback)
        && isDifferentialSensorsEqual(config1.DifferentialSensors, config2.DifferentialSensors)
        && isDifferentialConstantsEqual(
            config1.DifferentialConstants, config2.DifferentialConstants)
        && isOpenLoopRampsEqual(config1.OpenLoopRamps, config2.OpenLoopRamps)
        && isClosedLoopRampsEqual(config1.ClosedLoopRamps, config2.ClosedLoopRamps)
        && isHardwareLimitSwitchEqual(config1.HardwareLimitSwitch, config2.HardwareLimitSwitch)
        && isAudioEqual(config1.Audio, config2.Audio)
        && isSoftwareLimitSwitchEqual(config1.SoftwareLimitSwitch, config2.SoftwareLimitSwitch)
        && isMotionMagicEqual(config1.MotionMagic, config2.MotionMagic)
        && isCustomParamsEqual(config1.CustomParams, config2.CustomParams)
        && isClosedLoopGeneralEqual(config1.ClosedLoopGeneral, config2.ClosedLoopGeneral)
        && isSlot0Equal(config1.Slot0, config2.Slot0)
        && isSlot1Equal(config1.Slot1, config2.Slot1)
        && isSlot2Equal(config1.Slot2, config2.Slot2);
  }

  private static boolean isMotorOutputEqual(MotorOutputConfigs obj1, MotorOutputConfigs obj2) {
    return obj1.Inverted.value == obj2.Inverted.value
        && obj1.NeutralMode.value == obj2.NeutralMode.value
        && fpEqual(obj1.DutyCycleNeutralDeadband, obj2.DutyCycleNeutralDeadband)
        && fpEqual(obj1.PeakForwardDutyCycle, obj2.PeakForwardDutyCycle)
        && fpEqual(obj1.PeakReverseDutyCycle, obj2.PeakReverseDutyCycle);
  }

  private static boolean isCurrentLimitEqual(CurrentLimitsConfigs obj1, CurrentLimitsConfigs obj2) {
    return fpEqual(obj1.StatorCurrentLimit, obj2.StatorCurrentLimit)
        && obj1.StatorCurrentLimitEnable == obj2.StatorCurrentLimitEnable
        && fpEqual(obj1.SupplyCurrentLimit, obj2.SupplyCurrentLimit)
        && obj1.SupplyCurrentLimitEnable == obj2.SupplyCurrentLimitEnable
        && fpEqual(obj1.SupplyCurrentThreshold, obj2.SupplyCurrentThreshold)
        && fpEqual(obj1.SupplyTimeThreshold, obj2.SupplyTimeThreshold);
  }

  private static boolean isVoltageEqual(VoltageConfigs obj1, VoltageConfigs obj2) {
    return fpEqual(obj1.SupplyVoltageTimeConstant, obj2.SupplyVoltageTimeConstant)
        && fpEqual(obj1.PeakForwardVoltage, obj2.PeakForwardVoltage)
        && fpEqual(obj1.PeakReverseVoltage, obj2.PeakReverseVoltage);
  }

  private static boolean isTorqueCurrentEqual(
      TorqueCurrentConfigs obj1, TorqueCurrentConfigs obj2) {
    return fpEqual(obj1.PeakForwardTorqueCurrent, obj2.PeakForwardTorqueCurrent)
        && fpEqual(obj1.PeakReverseTorqueCurrent, obj2.PeakReverseTorqueCurrent)
        && fpEqual(obj1.TorqueNeutralDeadband, obj2.TorqueNeutralDeadband);
  }

  private static boolean isFeedbackEqual(FeedbackConfigs obj1, FeedbackConfigs obj2) {
    return fpEqual(obj1.FeedbackRotorOffset, obj2.FeedbackRotorOffset)
        && fpEqual(obj1.SensorToMechanismRatio, obj2.SensorToMechanismRatio)
        && fpEqual(obj1.RotorToSensorRatio, obj2.RotorToSensorRatio)
        && obj1.FeedbackSensorSource.value == obj2.FeedbackSensorSource.value
        && obj1.FeedbackRemoteSensorID == obj2.FeedbackRemoteSensorID;
  }

  private static boolean isDifferentialSensorsEqual(
      DifferentialSensorsConfigs obj1, DifferentialSensorsConfigs obj2) {
    return obj1.DifferentialSensorSource.value == obj2.DifferentialSensorSource.value
        && obj1.DifferentialTalonFXSensorID == obj2.DifferentialTalonFXSensorID
        && obj1.DifferentialRemoteSensorID == obj2.DifferentialRemoteSensorID;
  }

  private static boolean isDifferentialConstantsEqual(
      DifferentialConstantsConfigs obj1, DifferentialConstantsConfigs obj2) {
    return fpEqual(obj1.PeakDifferentialDutyCycle, obj2.PeakDifferentialDutyCycle)
        && fpEqual(obj1.PeakDifferentialVoltage, obj2.PeakDifferentialVoltage)
        && fpEqual(obj1.PeakDifferentialTorqueCurrent, obj2.PeakDifferentialTorqueCurrent);
  }

  private static boolean isOpenLoopRampsEqual(
      OpenLoopRampsConfigs obj1, OpenLoopRampsConfigs obj2) {
    return fpEqual(obj1.DutyCycleOpenLoopRampPeriod, obj2.DutyCycleOpenLoopRampPeriod)
        && fpEqual(obj1.VoltageOpenLoopRampPeriod, obj2.VoltageOpenLoopRampPeriod)
        && fpEqual(obj1.TorqueOpenLoopRampPeriod, obj2.TorqueOpenLoopRampPeriod);
  }

  private static boolean isClosedLoopRampsEqual(
      ClosedLoopRampsConfigs obj1, ClosedLoopRampsConfigs obj2) {
    return fpEqual(obj1.DutyCycleClosedLoopRampPeriod, obj2.DutyCycleClosedLoopRampPeriod)
        && fpEqual(obj1.VoltageClosedLoopRampPeriod, obj2.VoltageClosedLoopRampPeriod)
        && fpEqual(obj1.TorqueClosedLoopRampPeriod, obj2.TorqueClosedLoopRampPeriod);
  }

  private static boolean isHardwareLimitSwitchEqual(
      HardwareLimitSwitchConfigs obj1, HardwareLimitSwitchConfigs obj2) {
    return obj1.ForwardLimitType.value == obj2.ForwardLimitType.value
        && obj1.ForwardLimitAutosetPositionEnable == obj2.ForwardLimitAutosetPositionEnable
        && fpEqual(obj1.ForwardLimitAutosetPositionValue, obj2.ForwardLimitAutosetPositionValue)
        && obj1.ForwardLimitEnable == obj2.ForwardLimitEnable
        && obj1.ForwardLimitSource.value == obj2.ForwardLimitSource.value
        && obj1.ForwardLimitRemoteSensorID == obj2.ForwardLimitRemoteSensorID
        && obj1.ReverseLimitType.value == obj2.ReverseLimitType.value
        && obj1.ReverseLimitAutosetPositionEnable == obj2.ReverseLimitAutosetPositionEnable
        && fpEqual(obj1.ReverseLimitAutosetPositionValue, obj2.ReverseLimitAutosetPositionValue)
        && obj1.ReverseLimitEnable == obj2.ReverseLimitEnable
        && obj1.ReverseLimitSource.value == obj2.ReverseLimitSource.value
        && obj1.ReverseLimitRemoteSensorID == obj2.ReverseLimitRemoteSensorID;
  }

  private static boolean isAudioEqual(AudioConfigs obj1, AudioConfigs obj2) {
    return obj1.BeepOnBoot == obj2.BeepOnBoot
        && obj1.BeepOnConfig == obj2.BeepOnConfig
        && obj1.AllowMusicDurDisable == obj2.AllowMusicDurDisable;
  }

  private static boolean isSoftwareLimitSwitchEqual(
      SoftwareLimitSwitchConfigs obj1, SoftwareLimitSwitchConfigs obj2) {
    return obj1.ForwardSoftLimitEnable == obj2.ForwardSoftLimitEnable
        && obj1.ReverseSoftLimitEnable == obj2.ReverseSoftLimitEnable
        && fpEqual(obj1.ForwardSoftLimitThreshold, obj2.ForwardSoftLimitThreshold)
        && fpEqual(obj1.ReverseSoftLimitThreshold, obj2.ReverseSoftLimitThreshold);
  }

  private static boolean isMotionMagicEqual(MotionMagicConfigs obj1, MotionMagicConfigs obj2) {
    return fpEqual(obj1.MotionMagicCruiseVelocity, obj2.MotionMagicCruiseVelocity)
        && fpEqual(obj1.MotionMagicAcceleration, obj2.MotionMagicAcceleration)
        && fpEqual(obj1.MotionMagicJerk, obj2.MotionMagicJerk)
        && fpEqual(obj1.MotionMagicExpo_kV, obj2.MotionMagicExpo_kV)
        && fpEqual(obj1.MotionMagicExpo_kA, obj2.MotionMagicExpo_kA);
  }

  private static boolean isCustomParamsEqual(CustomParamsConfigs obj1, CustomParamsConfigs obj2) {
    return fpEqual(obj1.CustomParam0, obj2.CustomParam0)
        && fpEqual(obj1.CustomParam1, obj2.CustomParam1);
  }

  private static boolean isClosedLoopGeneralEqual(
      ClosedLoopGeneralConfigs obj1, ClosedLoopGeneralConfigs obj2) {
    return obj1.ContinuousWrap == obj2.ContinuousWrap;
  }

  private static boolean isSlot0Equal(Slot0Configs obj1, Slot0Configs obj2) {
    return fpEqual(obj1.kP, obj2.kP)
        && fpEqual(obj1.kI, obj2.kI)
        && fpEqual(obj1.kD, obj2.kD)
        && fpEqual(obj1.kS, obj2.kS)
        && fpEqual(obj1.kV, obj2.kV)
        && fpEqual(obj1.kA, obj2.kA)
        && fpEqual(obj1.kG, obj2.kG)
        && obj1.GravityType.value == obj2.GravityType.value
        && obj1.StaticFeedforwardSign.value == obj2.StaticFeedforwardSign.value;
  }

  private static boolean isSlot1Equal(Slot1Configs obj1, Slot1Configs obj2) {
    return fpEqual(obj1.kP, obj2.kP)
        && fpEqual(obj1.kI, obj2.kI)
        && fpEqual(obj1.kD, obj2.kD)
        && fpEqual(obj1.kS, obj2.kS)
        && fpEqual(obj1.kV, obj2.kV)
        && fpEqual(obj1.kA, obj2.kA)
        && fpEqual(obj1.kG, obj2.kG)
        && obj1.GravityType.value == obj2.GravityType.value
        && obj1.StaticFeedforwardSign.value == obj2.StaticFeedforwardSign.value;
  }

  private static boolean isSlot2Equal(Slot2Configs obj1, Slot2Configs obj2) {
    return fpEqual(obj1.kP, obj2.kP)
        && fpEqual(obj1.kI, obj2.kI)
        && fpEqual(obj1.kD, obj2.kD)
        && fpEqual(obj1.kS, obj2.kS)
        && fpEqual(obj1.kV, obj2.kV)
        && fpEqual(obj1.kA, obj2.kA)
        && fpEqual(obj1.kG, obj2.kG)
        && obj1.GravityType.value == obj2.GravityType.value
        && obj1.StaticFeedforwardSign.value == obj2.StaticFeedforwardSign.value;
  }
}
