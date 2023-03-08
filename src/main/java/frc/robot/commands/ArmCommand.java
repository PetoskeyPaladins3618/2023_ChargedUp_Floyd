// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsytem;


public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */

  

  public ArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_ArmSubsytem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.m_ArmSubsytem.armLimitSwitch.get() && RobotContainer.m_joystick.getRawAxis(1) < 0) {
      ArmSubsytem.arm.set(0);
    }
    else {
        if (RobotContainer.m_joystick.getRawAxis(1) > 0.1 || RobotContainer.m_joystick.getRawAxis(1) < -0.1) {
      ArmSubsytem.arm.set(RobotContainer.m_joystick.getRawAxis(1) * -1);
      }
      else {
      ArmSubsytem.arm.set(0);
      }
    }
    if(!RobotContainer.m_ArmSubsytem.armLimitSwitch.get() || RobotContainer.m_xboxController.getStartButton()) RobotContainer.m_ArmSubsytem.encoderOffset = ArmSubsytem.armEncoder.getAbsolutePosition();
    SmartDashboard.putBoolean("ArmLimitSwitch", !RobotContainer.m_ArmSubsytem.armLimitSwitch.get());
    SmartDashboard.putNumber("True Arm Position", ArmSubsytem.armEncoder.getAbsolutePosition() - RobotContainer.m_ArmSubsytem.encoderOffset);
   
    // if (RobotContainer.m_joystick.getRawButton(5)) {
      
    // }
    /* 
     if (RobotContainer.m_joystick.getRawButton(1))  {
      ArmSubsytem.arm.set(1);
    }
    else {
      ArmSubsytem.arm.set(0);
    } 
    */
    // if (RobotContainer.m_ArmSubsytem.armLimitSwitch = true)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
