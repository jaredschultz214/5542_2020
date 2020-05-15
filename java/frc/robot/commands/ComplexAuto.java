/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;


import frc.robot.Constants.*;
import frc.robot.subsystems.*;


public class ComplexAuto extends SequentialCommandGroup {
  /**
   * Creates a new ComplexAuto.
   *
   * This was going to be the 6 ball auto
   * 
   * 
   * commands in auto run these methods in this order: (init, (Exicute, isFinished), End)
   *                                                              /\        | |       /\
   *                                                               \_FALSE_/   \_TRUE_/
   * Sequential Command group: what ever is in the perenthaseese will exicute in order 
   * Parallel Command group: what ever is in the perenthaseese will exicute at the same time weighting for everything's isFin method to return true
   * Parallel Race Command group: everything will exicute at the same time, but when anything's isFin method returns true the command ends
   * 
   *  
   */
  public ComplexAuto(DriveSubsystem drive, BallElevatorSubsystem ballElevator, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {

    addCommands(
        new SequentialCommandGroup(
            new AutoSetup(drive),// setup
            new LimeLightDrive(drive), //aim
            new ParallelCommandGroup(new Shoot_Auto(shooterSubsystem),// shoot
                new BallElevator_Auto(ballElevator)),
            new TurnDeg(0, .3 , drive)),// turn robot back to pointing down field
        new ParallelRaceGroup(new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,drive),
            new Intake_Auto(intakeSubsystem, .8), // drive forward, pick up balls, and index them
            new BallElevatorIndex_Auto(ballElevator)),
        new DriveDistance(AutoConstants.kAutoBackupDistanceInches, AutoConstants.kAutoBackDriveSpeed,drive),//Drive back to line
        new LimeLightDrive(drive),//Aim
        new ParallelCommandGroup(new Shoot_Auto(shooterSubsystem),//shoot
            new BallElevator_Auto(ballElevator)));
  }

}
