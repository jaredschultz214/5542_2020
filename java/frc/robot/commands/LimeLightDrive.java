/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import oi.limelightvision.limelight.frc.ControlMode.CamMode;
import oi.limelightvision.limelight.frc.ControlMode.LedMode;
import frc.robot.Constants.LimeLightConstants;
/**
 * WARNING. THIS CODE IS DENSE AND HIGH LEVEL
 * Look up PID loops first
 * This code takes over the drive train, uses the angle between the limelight camera and the target to center, and was supposed to move backword if it was too close
 * 
 * 
 */
public class LimeLightDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private double deg;
  private double lastTimeStamp;
  private double p;
  private double i;
  private double d;
  private double distance;
  private double linError;// these were used for the moving backword that didn't work
  private double linD;//////

  private double maxOutput = 1;


  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward   The control input for driving forwards/backwards
   * @param rotation  The control input for turning
   */
  public LimeLightDrive(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
    m_drive.lockMotors();
  }

  @Override
  public void execute() {
    maxOutput = m_drive.getCurrentSpeedLim();
    m_drive.setMaxOutput(.5);// so the robot doesn't go too fast and over shoot(or spase out too much)

      Robot.m_robotContainer.getLimeLight().setLEDMode(LedMode.kforceOn);// turn on light
      Robot.m_robotContainer.getLimeLight().setCamMode(CamMode.kvision);// turn on vision tracking mode

    p = deg;// last error
    d = (deg-Robot.m_robotContainer.getLimeLight().getdegRotationToTarget())/(lastTimeStamp-Timer.getFPGATimestamp());// Psudo derivitive of error
    i = i+Robot.m_robotContainer.getLimeLight().getdegRotationToTarget();// Psudo intigral of error
    deg = Robot.m_robotContainer.getLimeLight().getdegRotationToTarget();// get Degree distance from target
    
    linD = distance-(LimeLightConstants.kTargetHight-LimeLightConstants.kLimelightHightInches)/
      Math.tan(Math.toRadians(LimeLightConstants.kLimelightAngle + Robot.m_robotContainer.getLimeLight().getdegVerticalToTarget()))/lastTimeStamp-Timer.getFPGATimestamp();
    
    distance = (LimeLightConstants.kTargetHight-LimeLightConstants.kLimelightHightInches)/ // distance formula using trig (refference the comment in the constants class)
      Math.tan(Math.toRadians(LimeLightConstants.kLimelightAngle + Robot.m_robotContainer.getLimeLight().getdegVerticalToTarget()));
    linError = LimeLightConstants.kMinDistance-distance;// linear error
    lastTimeStamp = Timer.getFPGATimestamp();// set time stamp for derivative

// for many of these things order is neccesary(though I may not have it correct, this is what was on the robot)

   
   
    //if(distance>LimeLightConstants.kMinDistance)
    m_drive.arcadeDrive(0,LimeLightConstants.kP*p+LimeLightConstants.kD*d+LimeLightConstants.kI*i);
    //else
    //m_drive.arcadeDrive(m_drive.getInversion()*.1,//(LimeLightConstants.kLinP*linError+LimeLightConstants.kLinD*linD),
    //    LimeLightConstants.kP*p+LimeLightConstants.kD*d+LimeLightConstants.kI*i);  
    }
    public void end(boolean interrupted) {
      System.out.println("LimEnd");// trouble shooting
      m_drive.arcadeDrive(0, 0);// stop the drive train from drifting off desired aim
      m_drive.setMaxOutput(maxOutput);
    }
    public boolean isFinished() {
      System.out.println("LimIsFin");// trouble shooting

      if(Math.abs(deg) <= LimeLightConstants.kLimeLightTolerance && Robot.m_robotContainer.getLimeLight().getIsTargetFound())// you NEED the isTargetFound because if there isn't a target the getDeg = 0.0.  This may be the reason why the autonomous aim is dumb
      {
        Robot.m_robotContainer.getLimeLight().setLEDMode(LedMode.kforceOff);// stop the eye death
        Robot.m_robotContainer.getLimeLight().setCamMode(CamMode.kdriver);
        m_drive.setMaxOutput(maxOutput);
        
        return true;
      }
      return false;
    }
  }
