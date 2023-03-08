// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


//THIS CODE CAN BE USED TO GET THE CONES OR SQUARES WHILE IN FRONT OF THE CHARGING PAD


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShortHighDropCommand extends SequentialCommandGroup {
  /** Creates a new AutoShortHighDropCommand. */
  public AutoShortHighDropCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoArmMoveCommand(96));
    addCommands(new AutoDriveForwardCommand(24));
    addCommands(new AutoArmMoveCommand(96));
    //addCommands(new AutoGrabberDropCommand());
    addCommands(new AutoDriveBackwardCommand(21));
    //addCommands(new AutoGrabberCloseCommand());
    addCommands(new AutoArmMoveCommand(0));
  }
}
