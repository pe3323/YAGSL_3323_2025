

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  // Set the height of the elevator in inches from the floor.
  // this value is the height of the robot + elevator.
  
  public static final double LEVEL0_HEIGHT = 0;  // 6
  public static final double LEVEL05_HEIGHT= 8;
  public static final double LEVEL1_HEIGHT = 13; // 21
  public static final double LEVEL2_HEIGHT = 32; // 41.5
  public static final double LEVEL3_HEIGHT = 59.5; // 62

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }  

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }



    public static final double weight = 120;
  public static final class ElevatorConstants {
        
    public static final double degreePerRot = 2.05;
    public static final int ele1 = 9;
    public static final double gearRatio = 36;  // 100 -- motor : bar
    public static final double drumCircumferenceIn = 5.81; // C of rope reel
    public static final double robotHeight = 5.875;  // floor to top of robot base in inches
  
  }

  public static final class CoralConstants {
    public static final int coralMotorId = 11;
    public static final int coralPivotMotorId = 10;
    public static final int coralGear = 50   ;
    public static final int coralPivotGear = 20;
  }

  public static final class ClimberConstants {
    public static final int harpoonMotorId = 12;
    public static final int lockMotorId = 13;
    public static final int lockGear= 63;
   
  }

  public static final class AlgaeConstants {
    public static final int extenderMotorId = 14;
    public static final int grabberMotorId = 15;
    public static final int grabberGear = 1;

    public static final double maxAlgaePosition = 163;
    public static final double miniumumAlgaePostition = 0;
  }
}
