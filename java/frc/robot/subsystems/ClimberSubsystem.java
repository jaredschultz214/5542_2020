/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new WheelSpinnerSubsystem.
   */
  private WPI_TalonSRX small = new WPI_TalonSRX(ClimberConstants.kClimberPortSmall);
  private WPI_TalonSRX big = new WPI_TalonSRX(ClimberConstants.kClimberPortBig);

  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    // This method will be called once per scheduler run
  }
  public void up(double speed){
    small.set(-speed);
  }
  public void smallDown(double speed){
    small.set(speed);
  }
  public void bigDown(double speed){
    big.set(-speed);
  }
  public void stop(){
    small.set(0);
    big.set(0);
  }

}
