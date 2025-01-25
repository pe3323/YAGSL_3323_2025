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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class Climber extends SubsystemBase {

  private final SparkMax harpoon;
  private final SparkMax lock;
  /** Creates a new ExampleSubsystem. */
  public Climber() {


    
    harpoon= new SparkMax(ClimberConstants.harpoonMotorId, MotorType.kBrushless);
    lock= new SparkMax(ClimberConstants.lockMotorId, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();

    config
    //.inverted(true)
    .idleMode(IdleMode.kBrake);
    harpoon.getEncoder().setPosition(0);
    lock.getEncoder().setPosition(0);

    harpoon.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    lock.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  

  public void setHarpoonPosition(double position) { // raises the roof

    harpoon.getEncoder().setPosition(position);
  }    

  public void setLockPosition(double position) { // raises the roof

    lock.getEncoder().setPosition(position);
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
