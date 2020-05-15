/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BallElevatorSubsystem;
import frc.robot.Constants.BallElevatorConstants;
import edu.wpi.first.wpilibj.Timer;
public class BallElevator_Auto extends CommandBase {
  /**
   * 
   */
  private final BallElevatorSubsystem m_BallElevator;
  private double delay = 0;

  public BallElevator_Auto(BallElevatorSubsystem subsystem) {
    m_BallElevator = subsystem;
    addRequirements(m_BallElevator);
  }
  /*
    This command was only used for the shooting in auto
  */
  @Override
  public void initialize() {
    m_BallElevator.resetEncoder();// always reset encoders when initilized (some exceptions to this rule)
    delay = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - delay  >=.5)// weight .5 second to let flywheels spin up
    m_BallElevator.manualElevator(BallElevatorConstants.kShootingElevatorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_BallElevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_BallElevator.getEncoder()*BallElevatorConstants.kCPRToDistance) >= 10;// shoule be average encoder distance
  }
}
