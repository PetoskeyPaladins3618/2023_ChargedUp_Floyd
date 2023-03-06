// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */

  public static DoubleSolenoid grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
  public GrabberSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
