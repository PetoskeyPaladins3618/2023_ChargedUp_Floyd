// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DropWheelsSubsystem;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class TeleOpDriveCommand extends CommandBase {
  /** Creates a new TwoTeleOpDriveCommand. */
  public TeleOpDriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_driveSubsystem);
   // DriveSubsystem.leftBack.setInverted(true);
   // DriveSubsystem.leftFront.setInverted(true);
    // DriveSubsystem.rightBack.setInverted(true);
    // DriveSubsystem.rightFront.setInverted(true);
    DriveSubsystem.robotDrive.setSafetyEnabled(false);
    //DriveSubsystem.robotDrive.driveCartesian(0.2, 0.1, 0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.leftFront.setOpenLoopRampRate(0.1);
    DriveSubsystem.leftFront.setClosedLoopRampRate(0.1);
    DriveSubsystem.leftBack.setOpenLoopRampRate(0.1);
    DriveSubsystem.leftBack.setClosedLoopRampRate(0.1);
    DriveSubsystem.rightFront.setOpenLoopRampRate(0.1);
    DriveSubsystem.rightFront.setClosedLoopRampRate(0.1);
    DriveSubsystem.rightBack.setOpenLoopRampRate(0.1);
    DriveSubsystem.rightBack.setClosedLoopRampRate(0.1);
    DropWheelsSubsystem.leftDrop.setInverted(true); 
    // RobotContainer.m_driveSubsystem.rightFront.setInverted(true);
   DriveSubsystem.rightBack.setInverted(false);
   DriveSubsystem.rightFront.setInverted(true);
   DriveSubsystem.leftFront.setInverted(false);
   DriveSubsystem.leftBack.setInverted(true);

   // Sets Encoders to 0
   DriveSubsystem.leftBackEncoder.setPosition(0);
   DriveSubsystem.rightBackEncoder.setPosition(0);
   DriveSubsystem.leftFrontEncoder.setPosition(0);
   DriveSubsystem.rightFrontEncoder.setPosition(0);



   }

  //boolean isDrop = true;
  public double movementSpeed;
  public double strafeSpeed;
  public double turnSpeed;
  boolean wheelsDown = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // DriveSubsystem.robotDrive.driveCartesian(0.2, 0.1, 0.2);
   //isDrop = RobotContainer.m_xboxController.getBButton();
   //SmartDashboard.putBoolean("Is Drop Drive On", isDrop);

  //Drop Wheels Stuff
    if (RobotContainer.m_xboxController.getLeftBumperPressed()) {
      DropWheelsSubsystem.backDropWheels.set(Value.kForward);
      DropWheelsSubsystem.frontDropWheels.set(Value.kForward);
      wheelsDown = true;
    }
    if (RobotContainer.m_xboxController.getRightBumperPressed()) {
      DropWheelsSubsystem.backDropWheels.set(Value.kReverse);
      DropWheelsSubsystem.frontDropWheels.set(Value.kReverse);
      wheelsDown = false;
    }
    SmartDashboard.putBoolean("DropWheelsDown", wheelsDown);

  //Driving Stuff
   if (wheelsDown == false) {
    DriveSubsystem.robotDrive.driveCartesian(RobotContainer.m_xboxController.getLeftY() * movementSpeed, RobotContainer.m_xboxController.getRightX() * turnSpeed, RobotContainer.m_xboxController.getLeftX() * strafeSpeed );
   // DriveSubsystem.robotDrive.drivePolar(RobotContainer.m_xboxController.getLeftY() * movementSpeed, , 0);

  // Reset encoders to zero on start press
  if(RobotContainer.m_xboxController.getStartButton()) {
    DriveSubsystem.leftBackEncoder.setPosition(0);
    DriveSubsystem.rightBackEncoder.setPosition(0);
    DriveSubsystem.leftFrontEncoder.setPosition(0);
    DriveSubsystem.rightFrontEncoder.setPosition(0);
  }

  }
  else {
    DropWheelsSubsystem.dropWheelDrive.arcadeDrive(RobotContainer.m_xboxController.getLeftY() * 0.7 , RobotContainer.m_xboxController.getRightX() * -0.7);
  } 
  
   
   if ((RobotContainer.m_xboxController.getLeftY() >= 0.15) || (RobotContainer.m_xboxController.getLeftY() <= -.15)) {
    movementSpeed = -0.4;
  }
  else {
    movementSpeed = 0;
  }

  if ((RobotContainer.m_xboxController.getLeftX() >= .2) || (RobotContainer.m_xboxController.getLeftX() <= -.2)) {
    strafeSpeed = 0.4;  }
  else {
    strafeSpeed = 0;
  }
  
  if ((RobotContainer.m_xboxController.getRightX() >= .15) || (RobotContainer.m_xboxController.getRightX() <= -.15)) {
    turnSpeed = 0.4;
  }
  else {
    turnSpeed = 0;
  }
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
