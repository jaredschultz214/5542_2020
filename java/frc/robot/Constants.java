/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterInches = 6;
    public static final double kGearBoxReduction = 10.71;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR / kGearBoxReduction;
  }   
    
    

public static final class LimeLightConstants{

    public static final double kP = .098;
    public static final double kI = .002;
    public static final double kD = .0100;
    public static final double kLinP = .0020;
    public static final double kLinD = .00909;
    public static final double kLimelightAngle = 26.0;
    public static final double kLimelightHightInches = 31.5;
    public static final double kTargetHight = 98.25;
    public static final double kMinDistance = 120;
    public static final double kLimeLightTolerance = .15;
    /**
     *          
     *                        (TargetHight-LimelightHightInches)
     *  Distance = -----------------------------------------------------------
     *               tan((Radians) fixedLimelightAngle + VerticalDegreeseToTarget)
     * 
     * 
     */
  }
  public static final class IntakeConstants{
    public static final int kIntakePort = 14;
  }
  public static final class ShooterConstants {
    public static final int kLeftShooterPort = 10;
    public static final int kRightShooterPort = 6;
  }
  public static final class ClimberConstants{
    public static final int kClimberPortSmall = 11;
    public static final int kClimberPortBig = 4;
  }

  public static final class AutoConstants {
    public static final double kAutoDriveDistanceInches = 160;
    public static final double kAutoBackupDistanceInches = -160;
    public static final double kSimpleAutoDriveInches = 80;
    public static final double kAutoDriveSpeed = 0.5;
    public static final double kAutoBackDriveSpeed = -0.5;
    public static final double kAutoDriveTime = 3;
  }

  public static final class BallElevatorConstants{
    public static final int kBallElevatorPort = 5;
    public static final double kShootingElevatorSpeed = .675;//.6 works
    public static final double kIndexingSpeed =.3;
    public static final double kEncoderCPR = 2048;
    public static final double kGearReduction = 6;
    public static final double kIndexRotations = 1.45;
    public static final double kCPRToDistance = 1/(kGearReduction*kEncoderCPR);// equals one rotation
    public static final int kindexDistance = 118;//color sensor
  }


  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kShoterJoyPort = 0;
  }
}

