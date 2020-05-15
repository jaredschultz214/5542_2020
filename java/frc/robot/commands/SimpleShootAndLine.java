/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.Constants.*;
import frc.robot.subsystems.*;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives backward.
 */
public class SimpleShootAndLine extends SequentialCommandGroup {
  /**
   * 
   *
   * @param drive The drive subsystem this command will run on
   * 
   * 
   */
  public SimpleShootAndLine(DriveSubsystem drive, BallElevatorSubsystem ballElevator, ShooterSubsystem shooterSubsystem) {
    drive.resetEncoders();
    addCommands(
        new AutoSetup(drive),// setup
        new SequentialCommandGroup(
            new LimeLightDrive(drive),// aim
            new ParallelCommandGroup(new Shoot_Auto(shooterSubsystem),
                new BallElevator_Auto(ballElevator))),// shoot
        new DriveDistance(AutoConstants.kSimpleAutoDriveInches, AutoConstants.kAutoDriveSpeed,drive));// drive
  }

}
