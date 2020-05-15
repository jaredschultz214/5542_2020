/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BallElevatorSubsystem;

public class BallElevatorIndex_Auto extends CommandBase {
  /**
   * 
   */
  private final BallElevatorSubsystem m_BallElevator;
  private boolean indexBoolean;
  private boolean antiLoop;

  public BallElevatorIndex_Auto(BallElevatorSubsystem subsystem) {
    m_BallElevator = subsystem;
    addRequirements(m_BallElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /*
    This command was never tested and I don't know if it works, but it was ment for the 6 ball auto to alow for indexing
  */
  @Override
  public void initialize() {
    indexBoolean = true;
    antiLoop = true;
    m_BallElevator.stopElevator();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((m_BallElevator.getIndex())&&indexBoolean&&antiLoop)
    {
      m_BallElevator.resetEncoder();
      indexBoolean = false;
      antiLoop = false;
    }
    else if((m_BallElevator.getIndex())||!indexBoolean)
    {
      indexBoolean = m_BallElevator.indexElevator();
    }
    else
    m_BallElevator.stopElevator();
    if(!m_BallElevator.getIndex()&&!antiLoop)
    {
    antiLoop = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
