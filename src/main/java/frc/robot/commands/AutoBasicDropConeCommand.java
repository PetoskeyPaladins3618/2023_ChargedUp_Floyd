// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBasicDropConeCommand extends SequentialCommandGroup {
  /** Creates a new AutoBasicDropConeCommand. */
  public AutoBasicDropConeCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoArmMoveCommand(84));
    addCommands(new AutoDriveForwardCommand(10));
    addCommands(new AutoArmMoveCommand(84));
    addCommands(new AutoGrabberDropCommand());
    addCommands(new AutoDriveBackwardCommand(150));
    addCommands(new AutoGrabberCloseCommand());
    addCommands(new AutoArmMoveCommand(0));
  }
}
