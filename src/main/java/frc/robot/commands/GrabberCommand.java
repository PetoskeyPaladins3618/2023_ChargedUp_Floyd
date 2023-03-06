// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabberSubsystem;

public class GrabberCommand extends CommandBase {
  /** Creates a new GrabberCommand. */
  public GrabberCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_grabberSubsystem);
  }
   public boolean ClawOpen;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GrabberSubsystem.grabber.set(Value.kReverse);
    ClawOpen = false;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_joystick.getRawButton(2) && ClawOpen == false) {
      GrabberSubsystem.grabber.set(Value.kForward);
      ClawOpen = true;
    }
    if (RobotContainer.m_joystick.getRawButton(1) && ClawOpen) {
      GrabberSubsystem.grabber.set(Value.kReverse);
      ClawOpen = false;
    }
    SmartDashboard.putBoolean("IsClawOpen", ClawOpen);
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
