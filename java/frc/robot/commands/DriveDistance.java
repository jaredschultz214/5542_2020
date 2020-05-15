/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private double angle;//////////////

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveDistance(double inches, double speed, DriveSubsystem drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();// for percise distance control
    angle = m_drive.getGyro();// for angle control without doing trig
    m_drive.lockMotors();
  }
  public void execute() {
    m_drive.arcadeDrive(m_speed,-.01*(m_drive.getGyro()-angle));// drive with a speed and some gyro correction. in future use the encoder values and use the difference between the two values to correct error
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    if(m_speed>0)
    {
    return m_drive.getAverageEncoderDistance() >= m_distance;//needs to be false to loop
    }
    else
    {
    return m_drive.getAverageEncoderDistance() < m_distance;// for future refference. DON'T USE AVERAGE ENCODER DISTANCE
                                                            // it one encoder could be at 0 and the other at 100 and you reached your desired value of 50
    }
  }
}
