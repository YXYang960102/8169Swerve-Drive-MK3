// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AMPConstants;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.IDConstants;

public class AMPSubsystem extends SubsystemBase {
  private CANSparkMax ArmMotor = new CANSparkMax(IDConstants.kAMPArmMotorPort, MotorType.kBrushless);
  private CANSparkMax AMPCrawlMotor = new CANSparkMax(IDConstants.kAMPCrawlMotorPort, MotorType.kBrushless);
  private RelativeEncoder ArmEncoder = ArmMotor.getEncoder();
  // private SparkAbsoluteEncoder ArmAbsEncoder = ArmMotor.getAbsoluteEncoder();
  // private SparkPIDController ArmPIDcontroller = ArmMotor.getPIDController();

  public double ArmkP, ArmkI, ArmkD, ArmkIZone, ArmkFF, ArmMaxOutput, ArmMinOutput;

  /** Creates a new AMP. */
  public AMPSubsystem() {
    ArmMotor.setInverted(false);
    ArmMotor.setIdleMode(IdleMode.kBrake);

    AMPCrawlMotor.setInverted(false);
    AMPCrawlMotor.setIdleMode(IdleMode.kBrake);

    // ArmEncoder.setPosition(ArmAbsEncoder.getPosition());

    ArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    ArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // ArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) AMPConstants.kUPLimit);
    ArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) AMPConstants.kDownLimit);

    // ArmkP = AMPConstants.kP;
    // ArmkI = AMPConstants.kI;
    // ArmkD = AMPConstants.kD;
    // ArmkIZone = AMPConstants.kIZone;
    // ArmkFF = AMPConstants.kFF;
    // ArmMaxOutput = AMPConstants.kArmMaxOutput;
    // ArmMinOutput = AMPConstants.kArmMinOutput;

    // SmartDashboard.putNumber("Arm P Gain", ArmkP);
    // SmartDashboard.putNumber("Arm I Gain", ArmkI);
    // SmartDashboard.putNumber("Arm D Gain", ArmkD);
    // SmartDashboard.putNumber("Arm I Zone", ArmkIZone);
    // SmartDashboard.putNumber("Arm Feed Forward", ArmkFF);
    // SmartDashboard.putNumber("Arm Max Output", ArmMaxOutput);
    // SmartDashboard.putNumber("Arm Min Output", ArmMinOutput);

    // // set Arm Spark PID
    // ArmPIDcontroller.setP(ArmkP);
    // ArmPIDcontroller.setI(ArmkI);
    // ArmPIDcontroller.setD(ArmkD);
    // ArmPIDcontroller.setIZone(ArmkIZone);
    // ArmPIDcontroller.setFF(ArmkFF);
    // ArmPIDcontroller.setOutputRange(ArmMaxOutput, ArmMinOutput);
    // ArmPIDcontroller.setFeedbackDevice(ArmAbsEncoder);
    // ArmMotor.burnFlash();

  }

  // public double getArmAbsolutePosition() {
  //   return ArmAbsEncoder.getPosition();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Position", ArmEncoder.getPosition());
    // SmartDashboard.putNumber("Arm Abs Position", ArmAbsEncoder.getPosition());

  }

  // public void setAMP() {
  //   ArmPIDcontroller.setReference(AMPConstants.kAMPPosition, ControlType.kPosition);
  // }

  // public void setArmDefult() {
  //   ArmPIDcontroller.setReference(AMPConstants.kAMPDefultPosition, ControlType.kPosition);
  // }

  // public void ArmHold() {
  //   ArmPIDcontroller.setReference(getArmAbsolutePosition(), ControlType.kPosition);
  // }

  public void ArmUP() {
    ArmMotor.set(AMPConstants.kAMPArmMotorRate);
  }

  public void ArmDown() {
    ArmMotor.set(-AMPConstants.kAMPArmMotorRate);
  }

  public void Armstop() {
    ArmMotor.set(0);
  }

  public void AMPCrawlAutoFwd() {
    AMPCrawlMotor.set(AMPConstants.kAMPCrawlRateAutoFwd);
  }

  public void AMPCrawlFwd() {
    AMPCrawlMotor.set(AMPConstants.kAMPCrawlRateFwd);
  }

  public void AMPCrawlRev() {
    AMPCrawlMotor.set(-AMPConstants.kAMPCrawlRateRev);
  }

  public void AMPCrawlStop() {
    AMPCrawlMotor.set(0);
  }

}
