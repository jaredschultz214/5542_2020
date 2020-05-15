/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ShooterSubsystem;

public class Shoot_Auto extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final ShooterSubsystem m_shooter;
  double shootTime = 0;

  public Shoot_Auto(ShooterSubsystem subsystem) {
    m_shooter = subsystem;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTime = Timer.getFPGATimestamp();
    System.out.println(Timer.getFPGATimestamp());
  }
  
  @Override
  public void execute() {
    m_shooter.manualShoot(-1,true);// put limelight formula into this
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp()-shootTime>2.5;// shoot for 2.5 seconds to make sure all balls got out. Probably didn't need this...
                                                  // could have goten away with a parrallelRaceCommandGroup in the auto commands and gon off of the elevator, but I didn't
  }
}
