/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
public class TurnDeg extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_deg;
  private double m_speed;
  private double error;

  /**
   * Creates a new TurnDeg.
   * 
   * Never was successfully tested
   *
   * 
   * 
   */
  public TurnDeg(double deg, double speed, DriveSubsystem drive) { 
    m_deg = deg;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    System.out.println("DegInit");
    error = m_deg - m_drive.getGyro();
    m_drive.lockMotors();
  }
  public void execute() {
    m_drive.arcadeDrive(0, error*.1);
    System.out.println("DegExicute");
    System.out.println(error);
    error = m_deg - m_drive.getGyro();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
    System.out.println("DegEnd");
    m_drive.unlockMotors();
  }

  @Override
  public boolean isFinished() {
    System.out.println("DegIsFin"); 
    if(Math.abs(error)<.3)
    return true;
    return false;
    
  }
}
