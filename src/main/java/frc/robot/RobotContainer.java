// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoBasicDropConeCommand;
import frc.robot.commands.AutoDriveForwardCommand;
import frc.robot.commands.AutoLongHighDrop;
import frc.robot.commands.AutoShortHighDropCommand;
import frc.robot.commands.AutoShortMidDropCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.PracticeAutonomousCommand;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.DropWheelsSubsystem;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public final static DropWheelsSubsystem m_dropWheelsSubsystem = new DropWheelsSubsystem();
  public final static ArmSubsytem m_ArmSubsytem = new ArmSubsytem();
  public final static GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
  public final static CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

  //The robot's commands are defined here...
  //public TeleopDriveCommand m_TeleopDriveCommand = new TeleopDriveCommand();

  // Teleoperated Commands
  public TeleOpDriveCommand m_TeleopDriveCommand = new TeleOpDriveCommand();
  public ArmCommand m_ArmCommand = new ArmCommand();
  public GrabberCommand m_GrabberCommand = new GrabberCommand();

  // Autonomous Commands
  public PracticeAutonomousCommand m_AutonomousCommand = new PracticeAutonomousCommand();
  public AutoBasicDropConeCommand m_AutoBasicDropConeCommand = new AutoBasicDropConeCommand();
  public AutoLongHighDrop m_AutoLongHighDrop = new AutoLongHighDrop();
  public AutoShortHighDropCommand m_AutoShortHighDropCommand = new AutoShortHighDropCommand();
  public AutoShortMidDropCommand m_AutoShortMidDropCommand = new AutoShortMidDropCommand();


  // Replace with CommandPS4Controller or CommandJoystick if needed
 // private final CommandXboxController m_driverController =
  //    new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //Controllers (Xbox Controller and Flight Stick)
  public static XboxController m_xboxController = new XboxController(0);
  public static Joystick m_joystick = new Joystick(1);
  
  public static SendableChooser<Integer> m_autoChooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_autoChooser.setDefaultOption("Long High Drop", 1);
    m_autoChooser.addOption("Short High Drop", 2);
    m_autoChooser.addOption("Long Mid Drop", 3);
    m_autoChooser.addOption("Short Mid Drop", 4);
    SmartDashboard.putData("Autonomous Option Selected", m_autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    switch(Robot.AutonomousChosen) {
      case(1): return m_AutoLongHighDrop;
      case(2): return m_AutoShortHighDropCommand;
      case(3): return m_AutoBasicDropConeCommand;
      case(4): return m_AutoShortMidDropCommand;
      default: return m_AutoLongHighDrop;
    }
  }
}
