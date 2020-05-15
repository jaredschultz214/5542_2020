/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
private WPI_TalonFX talonFX1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
private WPI_TalonFX talonFX2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
private WPI_TalonFX talonFX3 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
private WPI_TalonFX talonFX4 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(talonFX1, talonFX2);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(talonFX3,talonFX4);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private Gyro m_gyro = new ADIS16448_IMU();

  // The left-side drive encoder
  double leftEncoderPosition = 0;
  double rightEncoderPosition = 0;
  double speedLim = 1;
  // The right-side drive encoder

  private int invertDirection = 1;
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    m_drive.setMaxOutput(.5);
    // Sets the distance per pulse for the encoders
  }
  public void periodic() {
    rightEncoderPosition = talonFX3.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse;
    leftEncoderPosition = talonFX1.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse;
    SmartDashboard.putNumber("encoder(in)L", leftEncoderPosition);
    SmartDashboard.putNumber("encoder(in)R", rightEncoderPosition);
    SmartDashboard.putNumber("LimeLightDeg", Robot.m_robotContainer.getLimeLight().getdegRotationToTarget());
    SmartDashboard.putNumber("LimeLightHight", Robot.m_robotContainer.getLimeLight().getdegVerticalToTarget());
    SmartDashboard.putNumber("Distance", (LimeLightConstants.kTargetHight-LimeLightConstants.kLimelightHightInches)/
      Math.tan(Math.toRadians(LimeLightConstants.kLimelightAngle + Robot.m_robotContainer.getLimeLight().getdegVerticalToTarget())));
    SmartDashboard.putNumber("DriveSpeed", speedLim);
    SmartDashboard.putNumber("gyro", getGyro());
      // This method will be called once per scheduler run
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(invertDirection*fwd, .75*rot);
  }
  public void tankDrive(double left, double right){// this can be used to control individual sides of the drive train
    m_leftMotors.set(left);
    m_rightMotors.set(right);
  }
  public double getGyro(){
  return m_gyro.getAngle();
  }
  public void resetGyro()
  {
    m_gyro.reset();
  }

  public void invertDirection(){
    invertDirection = -1;
  }
  public void resetInversion(){
    invertDirection = 1;
  }

  public double getRightSpeed(){
    return -talonFX3.getSelectedSensorVelocity()/DriveConstants.kEncoderDistancePerPulse;
  }
  public double getLeftSpeed(){
    return talonFX1.getSelectedSensorVelocity()/DriveConstants.kEncoderDistancePerPulse;
  }
  public void resetEncoders() {
    talonFX1.setSelectedSensorPosition(0);//m_leftEncoder.reset();
    talonFX3.setSelectedSensorPosition(0);//m_rightEncoder.reset();
  }
  public double getAverageEncoderDistance() {
    return (-rightEncoderPosition + leftEncoderPosition) / 2.0;
  }
  public double getLeftEncoder() {// raw ticks
    return leftEncoderPosition;
  }
  public double getRightEncoder() {
    return -rightEncoderPosition;
  }
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  public void calculateMaxOutput(double maxOutput)// used to store the max output value
  {
    speedLim = maxOutput;
  }
  public int getInversion(){
    return invertDirection;
  }
  public double getCurrentSpeedLim(){
    return speedLim;
  }
  public void lockMotors(){// can use this for anti-Defence
talonFX1.setNeutralMode(NeutralMode.Brake);
talonFX2.setNeutralMode(NeutralMode.Brake);
talonFX3.setNeutralMode(NeutralMode.Brake);
talonFX4.setNeutralMode(NeutralMode.Brake);
  }
  public void unlockMotors(){
talonFX1.setNeutralMode(NeutralMode.Coast);
talonFX2.setNeutralMode(NeutralMode.Coast);
talonFX3.setNeutralMode(NeutralMode.Coast);
talonFX4.setNeutralMode(NeutralMode.Coast);    
  }
}
