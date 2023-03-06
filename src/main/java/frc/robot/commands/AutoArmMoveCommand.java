// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsytem;

public class AutoArmMoveCommand extends CommandBase {
  /** Creates a new AutoArmMoveCommand. */
  private double ArmEncoderValue;
  private boolean up;

  public AutoArmMoveCommand(double Degrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    ArmEncoderValue = -Degrees * Constants.ENCODER_CONVERSION_TO_DEGREES;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ArmEncoderValue > ArmSubsytem.armEncoder.getAbsolutePosition()- RobotContainer.m_ArmSubsytem.encoderOffset) {
      up = true;
    }
    else {
      up = false;
    }

    if (up)  {
      ArmSubsytem.arm.set(1);
    }
    else {
      ArmSubsytem.arm.set(-1);
    }
    SmartDashboard.putNumber("Goal", ArmEncoderValue);
    SmartDashboard.putNumber("Current", ArmSubsytem.armEncoder.getAbsolutePosition()- RobotContainer.m_ArmSubsytem.encoderOffset);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsytem.arm.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ArmEncoderValue - 0.011) < ArmSubsytem.armEncoder.getAbsolutePosition() - RobotContainer.m_ArmSubsytem.encoderOffset && ((ArmEncoderValue + 0.011) > ArmSubsytem.armEncoder.getAbsolutePosition() - RobotContainer.m_ArmSubsytem.encoderOffset) ;
  }
  //&& ((ArmEncoderValue + 0.011) > ArmSubsytem.armEncoder.getAbsolutePosition())
}
