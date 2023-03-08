// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.RobotContainer; 
//import frc.robot.RobotContainer;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
public static CANSparkMax leftBack = new CANSparkMax(Constants.LEFT_BACK, MotorType.kBrushless);
public static CANSparkMax rightBack = new CANSparkMax(Constants.RIGHT_BACK, MotorType.kBrushless);
public static CANSparkMax leftFront = new CANSparkMax(Constants.LEFT_FRONT, MotorType.kBrushless);
public static CANSparkMax rightFront = new CANSparkMax(Constants.RIGHT_FRONT, MotorType.kBrushless);
public ADIS16470_IMU gyro = new ADIS16470_IMU();
//public static CANSparkMax leftDrop = new CANSparkMax(Constants.LEFT_DROP, MotorType.kBrushless);
//public static CANSparkMax rightDrop = new CANSparkMax(Constants.RIGHT_DROP, MotorType.kBrushless);

public static RelativeEncoder leftBackEncoder = leftBack.getEncoder();
public static RelativeEncoder rightBackEncoder = rightBack.getEncoder();
public static RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
public static RelativeEncoder rightFrontEncoder = rightFront.getEncoder();



// Averages every encoder except Right Front
public static double encoderAverager() {
  double encoderAverage = (Math.abs(leftBackEncoder.getPosition()) + Math.abs(rightBackEncoder.getPosition()) + Math.abs(leftFrontEncoder.getPosition())) / 3;
  return encoderAverage;
} 


public static MecanumDrive robotDrive = new MecanumDrive(rightFront, rightBack, leftFront, leftBack);
//public static DifferentialDrive dropWheelDrive = new DifferentialDrive(leftDrop , rightDrop);


//hello

//6448_IMU gyro = new ADIS16448_IMU();
public static final String MecanumDrive = null;

private final AnalogInput ultrasonic = new AnalogInput(0);





/**
 * 
 */

public DriveSubsystem() {
  //clearStickyFaults();
  //gyro.set();
  robotDrive.setSafetyEnabled(false);
  //dropWheelDrive.setSafetyEnabled(false);
  // leftFront.setInverted(true);
  // leftBack.setInverted(true);
  // rightFront.setInverted(true);
  }

  @Override
  public void periodic() {

    //DriveSubsystem.robotDrive.driveCartesian(RobotContainer.wheelsGoBrr.getLeftY() * 0.3, RobotContainer.wheelsGoBrr.getLeftX() * -0.3, RobotContainer.wheelsGoBrr.getRightX() * 0.3);
    // This method will be called once per scheduler run
    double rawUltrasonicValue = ultrasonic.getValue();

    //voltage_scale_factor allows us to compensate for differences in supply voltage.

    double voltage_scale_factor = 5/RobotController.getVoltage5V();

    //gryo stuff
      //double gyroRawAngle = gyro.getAngle();
    
    //double currentDistanceCentimeters = rawUltrasonicValue * voltage_scale_factor * 0.12;

    double currentDistanceInches = rawUltrasonicValue * voltage_scale_factor * 0.0482;

    double currentDistanceFeet = rawUltrasonicValue * voltage_scale_factor * 0.0040166;
    //ultrasonic sensor values smart dashboard
    //  SmartDashboard.putNumber("DistanceCentimeters", currentDistanceCentimeters);
     // SmartDashboard.putNumber("Ultrasonic Voltage", rawUltrasonicValue);
      SmartDashboard.putNumber("DistanceInches", currentDistanceInches);
      SmartDashboard.putNumber("DistanceFeet", currentDistanceFeet);

      SmartDashboard.putNumber("Left Back Encoder", leftBackEncoder.getPosition());
      SmartDashboard.putNumber("Right Back Encoder", rightBackEncoder.getPosition());
      SmartDashboard.putNumber("Left Front Encoder", leftFrontEncoder.getPosition());
      SmartDashboard.putNumber("Right Front Encoder", rightFrontEncoder.getPosition());

      SmartDashboard.putNumber("Encoder Average", encoderAverager());



      //SmartDashboard.putNumber("LeftX", RobotContainer.m_xboxController.getLeftX());
      //SmartDashboard.putNumber("LeftY", RobotContainer.m_xboxController.getLeftY());
      SmartDashboard.putNumber("RightX", RobotContainer.m_xboxController.getRightX());
    //Gyro Values Smart Dashboards
      //SmartDashboard.putNumber(key: "RobotAngle", gyroRawAngle);
  }
}
