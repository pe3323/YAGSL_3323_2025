// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final SparkMax elevator1;
  //private final SparkMax elevator2;
  private int level = 0;
  private PIDController heightController;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {

    heightController = new PIDController(.05, 0, 0);
    heightController.setTolerance(0.1);
    
    elevator1= new SparkMax(ElevatorConstants.ele2, MotorType.kBrushless);
    elevator1.clearFaults();
    //elevator1= new SparkMax(ElevatorConstants.ele1, MotorType.kBrushless);
    //elevator1.clearFaults();

    SparkMaxConfig config = new SparkMaxConfig();  
    SoftLimitConfig softLimit = new SoftLimitConfig();
    SparkMaxConfig leftconfig = new SparkMaxConfig();  
    SparkMaxConfig rightconfig = new SparkMaxConfig();  

    softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(100) // 500... turning it down for testing
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(0);
    
    config
    .smartCurrentLimit(105, 40)
    .idleMode(IdleMode.kBrake)
    .apply(softLimit);

    leftconfig
    .inverted(false);
    //.follow(ElevatorConstants.ele1);
  

    rightconfig
    .inverted(true);
    


    //elevator1.getEncoder().setPosition(0);
    //elevator1.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevator1.getEncoder().setPosition(0);
    elevator1.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  

  }


  public void raise() { // raises the roof

    elevator1.set(-.30);
    
  }

  public void lower() { // raises the roof

    elevator1.set(.30);
    
  }    

  public void stop(){
    elevator1.set(0);
    
  }
 public void setLevel ( int level) {
  SmartDashboard.putNumber("level", level);
  this.level = level;
 }
  public boolean atLevel0(){
    return level == 0;
  }

  public boolean atLevel1(){
    return level == 1;
  }

  public boolean atLevel2(){
    return level == 2;
  }

  public boolean atLevel3(){
    return level == 3;
  }

  public double getPosition() {
    return elevator1.getEncoder().getPosition();
  }

  public void setPosition(double position) { // raises the roof
    double targetPosition = ElevatorConstants.gearRatio*(position/ElevatorConstants.drumCircumferenceIn);
    elevator1.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
  }    



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void setSpeed(double i){
      elevator1.set(i * -1.0);
    
  }


  public double getHeight() {
    return elevator1.getEncoder().getPosition()* -1.0;
  }

  public PIDController getController() {
    return heightController;
  }
}
