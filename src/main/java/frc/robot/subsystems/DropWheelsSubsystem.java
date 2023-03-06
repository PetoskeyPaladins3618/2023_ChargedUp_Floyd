// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DropWheelsSubsystem extends SubsystemBase {
  /** Creates a new DropWheelsSubsystem. */
  public static CANSparkMax leftDrop = new CANSparkMax(Constants.LEFT_DROP, MotorType.kBrushless);
  public static CANSparkMax rightDrop = new CANSparkMax(Constants.RIGHT_DROP, MotorType.kBrushless);

  public static DifferentialDrive dropWheelDrive = new DifferentialDrive(rightDrop, leftDrop);

public Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
public static DoubleSolenoid backDropWheels = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 5);
public static DoubleSolenoid frontDropWheels = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 4);


  public DropWheelsSubsystem() {
    dropWheelDrive.setSafetyEnabled(false);

    
   // public void initDefaultCommand()  {
    //  setDefaultCommand(new TwoTeleOpDriveCommand);
   // }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
