/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;

import frc.robot.subsystems.ClimberSubsystem;

public class Climb extends CommandBase {
  /**
   * 
   */
  private final ClimberSubsystem m_climber;

  private final BooleanSupplier m_up;
  private final BooleanSupplier m_smallDown;
  private final BooleanSupplier m_bigDown;


  public Climb(ClimberSubsystem subsystem, BooleanSupplier up, BooleanSupplier smallDown, BooleanSupplier bigDown) {
    m_climber = subsystem;
    m_smallDown = smallDown;
    m_bigDown = bigDown;
    m_up = up;
    addRequirements(m_climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   if(m_up.getAsBoolean() == true)
      m_climber.up(.5);// found this value worked and wasn't super bouncey
      else
      {
      if(m_smallDown.getAsBoolean() == true)
      m_climber.smallDown(.4);// doesn't realy matter the value, but I found .4 was nice on the motor and climber arm
      if(m_bigDown.getAsBoolean() == true)
      m_climber.bigDown(1);// as fast a possible
      }
      if(!(m_up.getAsBoolean()||m_smallDown.getAsBoolean()||m_bigDown.getAsBoolean()))
      m_climber.stop();
  }
/*
    Not an Auto Command

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
