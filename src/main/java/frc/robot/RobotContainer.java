// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.autonomus;
import frc.robot.commands.climberCommand;
import frc.robot.commands.climberDownCommand;
//import frc.robot.commands.driveCommand;
import frc.robot.commands.intakeCommand;
//import frc.robot.commands.limelightTurnCommand;
import frc.robot.commands.outtakeCommand;
import frc.robot.commands.shooterCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.climberSub;
//import frc.robot.subsystems.driveSub;
import frc.robot.subsystems.intakeSub;
import frc.robot.subsystems.shooterSub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DataLog m_logger = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...

  private final SwerveDrive m_swerveDrive = new SwerveDrive();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);


  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);
  static PS4Controller testController = new PS4Controller(Constants.USB.testController);

  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[4];
  public Button xBoxLeftTrigger, xBoxRightTrigger;

  public RobotContainer() {
    initializeSubsystems();
    initializeAutoChooser();

    // Configure the button bindings
    configureButtonBindings();
   /**  UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(160, 120); //Usually (640,320)
    camera.setFPS(30);

    PortForwarder.add(5800, "10.80.6.12", 5800);
    PortForwarder.add(5801, "10.80.6.12", 5801);
    PortForwarder.add(5802, "10.80.6.12", 5802);
  }
*/
  }
  public void initializeSubsystems() {
    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> -testController.getLeftY(),
            () -> -testController.getLeftX(),
            () -> -testController.getRightX()));
    m_fieldSim.initSim();
  }

/**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 90));
      shooter.whileHeld(new shooterCommand());
      intake.whileHeld(new intakeCommand());
  intakeReverse.whileHeld(new outtakeCommand());
  climber.whileHeld(new climberCommand());
  climberDown.whileHeld(new climberDownCommand());

  }

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
    //return m_autoCommand;
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void autonomousInit() {}

  public void autonomousPeriodic() {}

  public void teleopInit() {}

  public void teleopPeriodic() {}

  public void simulationInit() {}

  public void simulationPeriodic() {}


public static shooterSub mShooterSub = new shooterSub();

public static climberSub mClimberSub = new climberSub();

public static intakeSub mIntakeSub = new intakeSub();

//public static PS4Controller driver = new PS4Controller(Constants.controller);

public static PS4Controller operator = new PS4Controller(Constants.operator);

JoystickButton intakeReverse = new JoystickButton(operator, 8);

JoystickButton intake = new JoystickButton(operator, 6);

JoystickButton shooter = new JoystickButton(operator, 5);

JoystickButton climber = new JoystickButton(operator, 1);

JoystickButton climberDown = new JoystickButton(operator, 3);

//JoystickButton limelight = new JoystickButton(driver, 6);

}





  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  /** Remove these public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    mDriveSub.setDefaultCommand(new driveCommand());
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(160, 120); //Usually (640,320)
    camera.setFPS(30);

    PortForwarder.add(5800, "10.80.6.12", 5800);
    PortForwarder.add(5801, "10.80.6.12", 5801);
    PortForwarder.add(5802, "10.80.6.12", 5802);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  /** Remove this private void configureButtonBindings() {
    shooter.whileHeld(new shooterCommand());
    intake.whileHeld(new intakeCommand());
intakeReverse.whileHeld(new outtakeCommand());
climber.whileHeld(new climberCommand());
climberDown.whileHeld(new climberDownCommand());
limelight.whileHeld(new limelightTurnCommand());
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /** public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
*/ //Remove these
