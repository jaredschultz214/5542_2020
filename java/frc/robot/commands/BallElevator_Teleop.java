/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.BallElevatorSubsystem;
import frc.robot.Constants.BallElevatorConstants;

public class BallElevator_Teleop extends CommandBase {
  /**
   * This command is slightly confusing and probably doesn't have the best coding practices in it
   */
  private final BallElevatorSubsystem m_BallElevator;
  private final BooleanSupplier m_Shoot;
  private final BooleanSupplier m_Index;
  private final BooleanSupplier m_reverse;
  private boolean indexBoolean;
  private int delay;
  private boolean antiLoop;

  public BallElevator_Teleop(BallElevatorSubsystem subsystem, BooleanSupplier shoot,
      BooleanSupplier index, BooleanSupplier reverse) {
    m_BallElevator = subsystem;
    m_Shoot = shoot;
    m_Index = index;
    m_reverse = reverse;
    addRequirements(m_BallElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexBoolean = true;
    antiLoop = true;
    m_BallElevator.stopElevator();

  }

  @Override
  public void execute() {
    if(((m_Index.getAsBoolean()||m_BallElevator.getIndex())&&indexBoolean)&&antiLoop)// true FIRST TIME when color sensor is in range or when the button is pressed
    {
      m_BallElevator.resetEncoder();// this is done so the index doesn't sum the encoder values
      indexBoolean = false;// both of these(in their own way) make sure the elevator doesn't index twice to the same input
      antiLoop = false;//////
    }
    else if((m_Index.getAsBoolean()||m_BallElevator.getIndex())||!indexBoolean)// only allows one pending index at a time
    {
      indexBoolean = m_BallElevator.indexElevator();// index elevator returns a boolean that when it has traveled a certain distance it sets indexBoolean to true
    }
    else if(m_Shoot.getAsBoolean())
    {
      if (delay>20)// looptime(about.02S) * 20 = about a .4S delay for the flywheels to spin up
      m_BallElevator.speedElevator(BallElevatorConstants.kShootingElevatorSpeed);
      delay++;
    }
    else if(m_reverse.getAsBoolean())
    {
    m_BallElevator.manualElevator(-.2);
    }
    else
    m_BallElevator.stopElevator();
    if(delay > 0 && !m_Shoot.getAsBoolean())
    delay = 0;
    if(!m_BallElevator.getIndex()&&!m_Index.getAsBoolean()&&!antiLoop)// reset antiloop so we are back where we started, weighting for another input via button or color sensor
    {
    antiLoop = true;
    }
  }


  /*
  Don't need these because this is not an autonomous command

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  */
}
