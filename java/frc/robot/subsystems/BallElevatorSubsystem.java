/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.BallElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class BallElevatorSubsystem extends SubsystemBase {
  /**
   * Creates a new BallElevatorSubsystem.
   */
  private WPI_TalonFX talonSRX = new WPI_TalonFX(BallElevatorConstants.kBallElevatorPort);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);// as far as I know you need these colors for the color sensor to compair to
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);// otherwise there is no boundarys and everything is yellow
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private boolean isYellow = false;


  public BallElevatorSubsystem() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget); 
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("raw encoder", talonSRX.getSelectedSensorPosition());  // don't do this, raw encoder values don't mean anything to anyone
    SmartDashboard.putNumber("elevator rotations", talonSRX.getSelectedSensorPosition()*BallElevatorConstants.kCPRToDistance);
    SmartDashboard.putNumber("IntakeDistance", m_colorSensor.getProximity());
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kYellowTarget) {
      isYellow = true;
    } else {
      isYellow = false;
    }
 
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
   // SmartDashboard.putNumber("red", detectedColor.red);                           coppied directly from color sensor class
   // SmartDashboard.putNumber("Green", detectedColor.green);
   // SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putBoolean("isYellow", isYellow);
    // This method will be called once per scheduler run
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    // This method will be called once per scheduler run
  }
  public void manualElevator(double speed){
    talonSRX.set(speed);
  }
  public void speedElevator(double speed){
    talonSRX.set(speed);
  }
  public boolean indexElevator(){
    if(talonSRX.getSelectedSensorPosition()*BallElevatorConstants.kCPRToDistance<BallElevatorConstants.kIndexRotations)
    {
    talonSRX.set(BallElevatorConstants.kIndexingSpeed);
    return false;
    }
    else
    {
      talonSRX.set(0);
    return true;
    }
  }
  public void stopElevator(){// ALWAYS include this method
  talonSRX.set(0);
  }
  public double getEncoder(){
    return talonSRX.getSelectedSensorPosition();
  }
  public void resetEncoder(){
    talonSRX.setSelectedSensorPosition(0);
  }
  public boolean getIndex(){
    return isYellow && m_colorSensor.getProximity()>BallElevatorConstants.kindexDistance;

}
}
