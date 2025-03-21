package frc.robot.subsystems.swervedrive;
    
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
    


    private final SparkMax coral;
    private DigitalInput sensor;
    SparkClosedLoopController pidController;

    private PIDController heightController;

    public Coral() {
        coral = new SparkMax(CoralConstants.coralMotorId, MotorType.kBrushless); // makes new motor controller that is
        SparkMaxConfig config = new SparkMaxConfig();// defined as the motor for the arm
        coral.clearFaults();
        heightController = new PIDController(.05, 0, 0); // .10
    //heightController.setTolerance(0.1);

        config
        .inverted(true)
        .idleMode(IdleMode.kBrake);
       coral.getEncoder().setPosition(0);


        config.closedLoop
            .outputRange(-1, 1);

       coral.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public boolean HasNote() {
        return sensor.get();

    }

    public void raise() { // raises the roof

        coral.set(-.30);
        SmartDashboard.putBoolean("Sensor Value", sensor.get());
        

    
    }

    // No more calamari
    public void lower() { // lowers the roof

        coral.set(.15);
        

    }

    public void stop() { // stops the roof
        coral.set(0);

    }

    public void setPosition(double position) { // lowers the roof

        coral.getEncoder().setPosition(position);
        

    }

    public double getposition() { // gets position
        return coral.getEncoder().getPosition();
    }
public double getHeight() {
  return coral.getEncoder().getPosition();
}

public PIDController getController() {
  return heightController;
}

public void setSpeed(double i) {
    coral.set(i);
}
}

