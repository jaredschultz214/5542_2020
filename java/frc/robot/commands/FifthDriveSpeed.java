/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

public class FifthDriveSpeed extends CommandBase {
  private final DriveSubsystem m_drive;

  public FifthDriveSpeed(DriveSubsystem drive) {
    m_drive = drive;
  }

  @Override
  public void initialize() {
    m_drive.setMaxOutput(0.2);
    m_drive.calculateMaxOutput(0.2);
  }

  @Override
  public void end(boolean interrupted) {
  }
}
