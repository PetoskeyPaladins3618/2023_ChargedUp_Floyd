// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveForwardCommand extends CommandBase {
  /** Creates a new AutoDriveForwardCommand. */

  private double finalEncoderValue;


  public AutoDriveForwardCommand(double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
    finalEncoderValue = inches / Constants.ENCODER_CONVERSION_TO_INCHES;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     // Sets Encoders to 0
   DriveSubsystem.leftBackEncoder.setPosition(0);
   DriveSubsystem.rightBackEncoder.setPosition(0);
   DriveSubsystem.leftFrontEncoder.setPosition(0);
   DriveSubsystem.rightFrontEncoder.setPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.robotDrive.driveCartesian(0.12, 0.019, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Resets encoder values and stops the robot
    DriveSubsystem.robotDrive.driveCartesian(0, 0, 0);
    DriveSubsystem.leftBackEncoder.setPosition(0);
    DriveSubsystem.rightBackEncoder.setPosition(0);
    DriveSubsystem.leftFrontEncoder.setPosition(0);
    DriveSubsystem.rightFrontEncoder.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return (DriveSubsystem.encoderAverager() >= finalEncoderValue);
  }
}
