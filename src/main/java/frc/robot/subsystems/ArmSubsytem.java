// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ArmSubsytem extends SubsystemBase {
  /** Creates a new ArmSubsytem. */
  
  public static CANSparkMax arm = new CANSparkMax(Constants.ARM, MotorType.kBrushed);
  public DigitalInput armLimitSwitch = new DigitalInput(Constants.ARM_LIMIT_SWITCH);
   public static DutyCycleEncoder armEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER);
   public double encoderOffset;
  
  public ArmSubsytem() {
    encoderOffset = ArmSubsytem.armEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic(
  ) {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("Arm Encoder", (armEncoder.getAbsolutePosition() -0.5)*-120 + 43.3);
    SmartDashboard.putNumber("Arm Degrees", -(armEncoder.getAbsolutePosition() - encoderOffset)/Constants.ENCODER_CONVERSION_TO_DEGREES);
    //SmartDashboard.putNumber("Joystick Axis Y", RobotContainer.m_joystick.getRawAxis(1));
    //SmartDashboard.putNumber("Arm Encoder", armEncoder.getPosition());
  }
}
