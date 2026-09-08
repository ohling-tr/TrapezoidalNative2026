// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private SparkMax m_motorArm;
  private SparkMaxConfig m_motorConfigArm;
  //private FeedForwardConfig m_armFeedForwardConfig;
  private RelativeEncoder m_encoderArm;
  private AbsoluteEncoder m_absEncoderArm;
  private SparkClosedLoopController m_closedLoopArm;
  private double m_ArmTargetPosition = armConstants.kArmUp;

  public ArmSubsystem() {
    m_motorArm = new SparkMax(armConstants.motorArm, MotorType.kBrushless);
    m_encoderArm = m_motorArm.getEncoder();
    m_absEncoderArm = m_motorArm.getAbsoluteEncoder();
    m_motorConfigArm = new SparkMaxConfig();
    m_motorConfigArm.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(armConstants.kCurrentLimit)
        .secondaryCurrentLimit(armConstants.kSecondaryCurrentLimit);
    m_motorConfigArm.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(0.15)
        .i(0)
        .d(0.0);
        //.outputRange(-.15, .15);
    m_motorConfigArm.closedLoop.feedForward
      .kCos(0.13)
      .kS(0.01);
    
    m_motorConfigArm.closedLoop.maxMotion
        .maxAcceleration(250)
        .cruiseVelocity(500)
        .allowedProfileError(1);
    m_closedLoopArm = m_motorArm.getClosedLoopController();
    
    m_motorConfigArm.absoluteEncoder
        .inverted(false)
        .positionConversionFactor(360);
    //m_SoftLimitArm = new SoftLimitConfig();
    //m_SoftLimitArm.forwardSoftLimit(armConstants.kFwdSoftLimit)
    //    .forwardSoftLimitEnabled(true)
    //    .reverseSoftLimit(armConstants.kRevSoftLimit)
    //    .reverseSoftLimitEnabled(true);
    //m_motorConfigArm.apply(m_SoftLimitArm);

    m_motorArm.configure(m_motorConfigArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setRelativeEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Postion", m_encoderArm.getPosition());
    SmartDashboard.putNumber("Arm Target", m_ArmTargetPosition);
    SmartDashboard.putNumber("Arm Abs Enc", m_absEncoderArm.getPosition());
    SmartDashboard.putNumber("Arm Current", m_motorArm.getOutputCurrent());
  }

  public void stopArmMotor() {
    m_motorArm.set(0);
  }

  public Command cmdStopArm() {
    return Commands.runOnce(() -> stopArmMotor(), this);
  }

  private void adjustArmPosition(boolean isAdjustUp) {
    setRelativeEncoder();
    if (isAdjustUp) {
      m_ArmTargetPosition = m_ArmTargetPosition + armConstants.kPostionAdjust;
    } else {
      m_ArmTargetPosition = m_ArmTargetPosition - armConstants.kPostionAdjust;
    }
    // if (m_ArmTargetPosition > armConstants.kFwdSoftLimit) {
    // m_ArmTargetPosition = armConstants.kFwdSoftLimit}
    // etc
    m_closedLoopArm.setSetpoint(m_ArmTargetPosition, ControlType.kMAXMotionPositionControl);
  }

  private void setArmPosition(double targetPosition) {
    setRelativeEncoder();
    m_ArmTargetPosition = targetPosition;
    m_closedLoopArm.setSetpoint(m_ArmTargetPosition, ControlType.kMAXMotionPositionControl);
  }

  public Command cmdSetArmUp() {
    return Commands.runOnce(() -> setArmPosition(armConstants.kArmUp), this);
  }

  public Command cmdSetArmDown() {
    return Commands.runOnce(() -> setArmPosition(armConstants.kArmDown), this);
  }

  public Command cmdAdjustArmPosition(boolean isAdjustUp) {
    return Commands.runOnce(() -> adjustArmPosition(isAdjustUp), this);
  }

  private void setRelativeEncoder() {
    m_encoderArm.setPosition(m_absEncoderArm.getPosition() / armConstants.kEncoderDegrees);
  }
}