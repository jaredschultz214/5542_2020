/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class LockDrive extends CommandBase {
  private final DriveSubsystem m_drive;

  public LockDrive(DriveSubsystem drive) {
    m_drive = drive;
  }

  @Override
  public void initialize() {
    m_drive.lockMotors();
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.unlockMotors();
  }
}
