/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import oi.limelightvision.limelight.frc.ControlMode.CamMode;
import oi.limelightvision.limelight.frc.ControlMode.LedMode;
import frc.robot.commands.LimeLightDrive;
/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class DefaultDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;
  private final BooleanSupplier m_shoot;
  private final Command driveWithLime;

  /**
   * Creates a new DefaultDrive.
   *
   * The bace teliop drive train command
   * 
   */
  public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation, BooleanSupplier shoot) {
    m_drive = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    m_shoot = shoot;
    driveWithLime = new LimeLightDrive(m_drive);// make 1 new limelight object to be used below
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    if(!m_shoot.getAsBoolean())
    {
    m_drive.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());// bace drive command
Robot.m_robotContainer.getLimeLight().setLEDMode(LedMode.kforceOff);// don't blind anyone
Robot.m_robotContainer.getLimeLight().setCamMode(CamMode.kdriver);// have no filter on camera so it can be used as a camera
m_drive.setMaxOutput(m_drive.getCurrentSpeedLim());
    }
    else{
      driveWithLime.execute();// I think this is techniquly bad codeing practice to call the exicute method, but I don't care
    }
  }
}
