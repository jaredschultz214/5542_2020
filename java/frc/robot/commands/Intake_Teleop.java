/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.IntakeSubsystem;

public class Intake_Teleop extends CommandBase {
  /**
   * Creates a new SpinWheel.
   */
  private final IntakeSubsystem m_Intake;
  private final DoubleSupplier m_Spin;
  private final DoubleSupplier m_ReverseSpin;


  public Intake_Teleop(IntakeSubsystem subsystem, DoubleSupplier spin, DoubleSupplier reverseSpin) {
    m_Intake = subsystem;
    m_Spin = spin;
    m_ReverseSpin = reverseSpin;
    addRequirements(m_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_ReverseSpin.getAsDouble()>0)
    m_Intake.manualSpin(-m_ReverseSpin.getAsDouble());// left trigger
    else
    m_Intake.manualSpin(m_Spin.getAsDouble());// right trigger
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
