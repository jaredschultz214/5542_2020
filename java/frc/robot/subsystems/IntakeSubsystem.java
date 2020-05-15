/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  /**
   * Creates a new WheelSpinnerSubsystem.
   */
  private WPI_TalonSRX talonSRX = new WPI_TalonSRX(IntakeConstants.kIntakePort);

  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    // This method will be called once per scheduler run
  }
  public void manualSpin(double speed){
    talonSRX.set(speed);
  }
  public void distanceSpin(double speed){// was never used, and was never going to be used. Get rid of stuff like this
    talonSRX.set(speed);
  }

}
