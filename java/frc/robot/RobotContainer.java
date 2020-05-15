/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import frc.robot.Constants.*;


import frc.robot.commands.ComplexAuto;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.SimpleShootAndLine;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.Shoot_Teleop;
import frc.robot.commands.HalveDriveSpeed;
import frc.robot.commands.Intake_Teleop;
import frc.robot.commands.LockDrive;
import frc.robot.commands.BallElevator_Teleop;
import frc.robot.commands.SwitchDriveDirection;
import frc.robot.commands.Climb;
import frc.robot.commands.FifthDriveSpeed;
import frc.robot.commands.FullDriveSpeed;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import oi.limelightvision.limelight.frc.LimeLight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final BallElevatorSubsystem m_BallElevatorSubsystem = new BallElevatorSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final LimeLight m_limeLight = new LimeLight();


  // The autonomous routines

  
  private final Command m_simpleAuto =
      new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
                        m_robotDrive);

  // A complex auto routine that shoots, drives backwards, picks up more balls, drives forwards, and shoots again.
  private final Command m_complexAuto = new ComplexAuto(m_robotDrive,m_BallElevatorSubsystem,m_shooterSubsystem,m_IntakeSubsystem);
  
  // A simple auto routine that shoots, and drives streight backwards
  private final Command m_SimpleShootAndLine = new SimpleShootAndLine(m_robotDrive,m_BallElevatorSubsystem,m_shooterSubsystem);
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_shooterJoy = new Joystick(OIConstants.kShoterJoyPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDrive(
            m_robotDrive,
            () -> m_driverController.getY(GenericHID.Hand.kLeft),
            () -> m_driverController.getX(GenericHID.Hand.kRight),
            () -> m_shooterJoy.getRawButton(2)));
  m_shooterSubsystem.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new Shoot_Teleop(
            m_shooterSubsystem,
            () -> m_shooterJoy.getThrottle(),
            () -> m_shooterJoy.getTrigger(),
            () -> m_shooterJoy.getRawButton(12)));

  m_IntakeSubsystem.setDefaultCommand(

        new Intake_Teleop(
          m_IntakeSubsystem,
          () -> m_driverController.getTriggerAxis(Hand.kRight),
          () -> m_driverController.getTriggerAxis(Hand.kLeft)));

m_ClimberSubsystem.setDefaultCommand(

          new Climb(
            m_ClimberSubsystem,
            () -> m_shooterJoy.getRawButton(3),
            () -> m_shooterJoy.getRawButton(4),
            () -> m_shooterJoy.getRawButton(6)));
m_BallElevatorSubsystem.setDefaultCommand(
            
          new BallElevator_Teleop(
            m_BallElevatorSubsystem,
            () -> m_shooterJoy.getTrigger(),
            () -> m_shooterJoy.getRawButton(11),
            () -> m_shooterJoy.getRawButton(9)));


    // Add commands to the autonomous command chooser
    m_chooser.addOption("SimpleLine", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);
    m_chooser.addOption("SimpleShootAndLine", m_SimpleShootAndLine);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Grab the hatch when the 'A' button is pressed.
    // While holding the shoulder button, drive at half speed
      new JoystickButton(m_driverController, Button.kA.value)// used as a rotating toggle
      .whenHeld(new HalveDriveSpeed(m_robotDrive));
    new JoystickButton(m_driverController, Button.kB.value)// used as a rotating toggle
    .whenHeld(new FifthDriveSpeed(m_robotDrive));
    new JoystickButton(m_driverController, Button.kX.value)// used as a rotating toggle
    .whenHeld(new FullDriveSpeed(m_robotDrive));
      new JoystickButton(m_driverController, Button.kBumperLeft.value)// used as a toggle
    .toggleWhenPressed(new SwitchDriveDirection(m_robotDrive));
    new JoystickButton(m_driverController, Button.kBumperRight.value)// used as a toggle
    .toggleWhenPressed(new LockDrive(m_robotDrive));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
  public LimeLight getLimeLight(){// allow commands to access the limelight
    return m_limeLight;
  }
}
