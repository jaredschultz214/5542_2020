/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.ShooterSubsystem;

public class Shoot_Teleop extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final ShooterSubsystem m_shooter;
  private final DoubleSupplier m_rotation;
  private final BooleanSupplier m_go;
  private final BooleanSupplier m_dump;

  public Shoot_Teleop(ShooterSubsystem subsystem, DoubleSupplier rotation, BooleanSupplier go, BooleanSupplier dump) {
    m_shooter = subsystem;
    m_rotation = rotation;
    m_go = go;
    m_dump = dump;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_dump.getAsBoolean())// dump button
    m_shooter.manualShoot(.2,true);
    else
    m_shooter.manualShoot(m_rotation.getAsDouble(),m_go.getAsBoolean());
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
