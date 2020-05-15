/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new shooter.
   */
  private WPI_TalonSRX talonSRX1 = new WPI_TalonSRX(ShooterConstants.kLeftShooterPort);
  private WPI_TalonSRX talonSRX2 = new WPI_TalonSRX(ShooterConstants.kRightShooterPort);
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void manualShoot(double speed, boolean go){
   speed = (-speed + 1)/4+.5;// equation to make the throtle go from .5 at bottom to 1 at the top
    if (go == true){
    talonSRX2.set(speed);
    talonSRX1.set(-speed);
  }
  else
  stopMotion();
  SmartDashboard.putNumber("Speed", speed);
}

  public void stopMotion(){
    talonSRX2.set(0);
    talonSRX1.set(0);
  }
}
