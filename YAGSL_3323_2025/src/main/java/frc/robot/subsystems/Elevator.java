// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;



import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

  private final SparkMax elevator1;
  private final SparkMax elevator2;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {


    
    elevator1= new SparkMax(ElevatorConstants.ele1, MotorType.kBrushless);
    elevator2= new SparkMax(ElevatorConstants.ele2, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    config
    //.inverted(true)
    .idleMode(IdleMode.kBrake);
    elevator1.getEncoder().setPosition(0);
    elevator2.getEncoder().setPosition(0);

    elevator1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevator2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  public void raise() { // raises the roof

    elevator1.set(-.30);
    elevator2.set(-.30);
  }

  public void lower() { // raises the roof

    elevator1.set(-.30);
    elevator2.set(-.30);
  }    

  public void stop(){
    elevator1.set(0);
    elevator2.set(0);
  }

  public void setPosition(double position) { // raises the roof

    elevator1.getEncoder().setPosition(ElevatorConstants.gearRatio*(position/ElevatorConstants.drumCircumferenceIn)+ElevatorConstants.robotHeight);
    elevator2.getEncoder().setPosition(ElevatorConstants.gearRatio*(position/ElevatorConstants.drumCircumferenceIn)+ElevatorConstants.robotHeight);
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
}
