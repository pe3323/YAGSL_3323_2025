// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class Climber extends SubsystemBase {

  public static final int EXTENDED = 1;
  public static final int RETRACTED = 0;
  private int state = RETRACTED;

  private final TalonFX harpoon;
  private final SparkMax lock;
 
  private PIDController lockController;
  private PIDController harpoonController;
  /** Creates a new ExampleSubsystem. */
  public Climber() {

    lockController = new PIDController(0.05, 0, 0);
    
    harpoon= new TalonFX(ClimberConstants.harpoonMotorId);
    lock= new SparkMax(ClimberConstants.lockMotorId, MotorType.kBrushless);
    
    TalonFXConfigurator harpoonConfig = harpoon.getConfigurator();
    SparkMaxConfig lockConfig = new SparkMaxConfig();

    SoftwareLimitSwitchConfigs harpoonLimits = new SoftwareLimitSwitchConfigs();

    harpoonLimits
    .withForwardSoftLimitEnable(true)
    .withReverseSoftLimitEnable(true)
    .withForwardSoftLimitThreshold(170)
    .withReverseSoftLimitThreshold(0);

 
harpoonConfig
.apply(harpoonLimits);


    lockConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
    

    
    
    lock.getEncoder().setPosition(0);
    

   

  
    lock.configure(lockConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
  }


  public void setState( int state ) {
    if ( state == EXTENDED ){
      HarpoonExtend();
    } 
    if ( state == RETRACTED ){
      HarpoonRetract();
    }
    this.state = state;
  }

  public int getState() {
    return state;
  }

  public void HarpoonExtend() {
    harpoon.set(1.0);
  }

  public void HarpoonRetract() {
    harpoon.set(-1.0);
  }



  public void setHarpoonPosition(double position) { // raises the roof

   
  }    

  public void setHarpoonSpeed(double i) {
    harpoon.set(i);
}

public void stopHarpoon() { // stops the roof
  harpoon.set(0);


}

public double getHarpoon() {
 return harpoon.getPosition().getValueAsDouble();
}

public PIDController getHarpoonController() {
  return harpoonController;
}


  public void setLockPosition(double position) { // raises the roof

    lock.getEncoder().setPosition(position);
  }    

  public void setLockSpeed(double i) {
    lock.set(i);
}

public void stopLock() { // stops the roof
  lock.set(0);

}

public double getLock() {
  return lock.getEncoder().getPosition();
}

public PIDController getLockController() {
  return lockController;
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
