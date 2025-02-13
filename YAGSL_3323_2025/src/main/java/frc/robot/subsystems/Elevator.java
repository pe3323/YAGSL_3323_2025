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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private final SparkMax elevator1;
  
  private PIDController heightController;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {

    heightController = new PIDController(0.1, 0, 0);
    heightController.setTolerance(0.1);
    
    elevator1= new SparkMax(ElevatorConstants.ele1, MotorType.kBrushless);
    
    SparkMaxConfig config = new SparkMaxConfig();
    SoftLimitConfig softLimit = new SoftLimitConfig();

    softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(72)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(0);
    
    config
    //.inverted(true)
    .idleMode(IdleMode.kBrake)
    .apply(softLimit);

   
    

    

    elevator1.getEncoder().setPosition(0);
   

    elevator1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  

  }


  public void raise() { // raises the roof

    elevator1.set(-.30);
    
  }

  public void lower() { // raises the roof

    elevator1.set(-.30);
    
  }    

  public void stop(){
    elevator1.set(0);
    
  }

  public double getPosition() {
    return elevator1.getEncoder().getPosition();
  }

  public void setPosition(double position) { // raises the roof

    System.out.println( "Setting the position to : " + position );
    double targetPosition = ElevatorConstants.gearRatio*(position/ElevatorConstants.drumCircumferenceIn)+ElevatorConstants.robotHeight;

    elevator1.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);


    elevator1.getEncoder().setPosition(targetPosition);
    
    
  }    
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


public void setSpeed(double i) {
    elevator1.set(i);
   
}


public double getHeight() {
  return elevator1.getEncoder().getPosition();
}

public PIDController getController() {
  return heightController;
}
}
