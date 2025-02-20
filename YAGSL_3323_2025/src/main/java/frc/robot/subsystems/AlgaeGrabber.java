package frc.robot.subsystems;
    
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeGrabber extends SubsystemBase {
          


    private final SparkMax grabber;
    private DigitalInput sensor;
    SparkClosedLoopController pidController;
    private PIDController poseController;

    public AlgaeGrabber() {
        grabber = new SparkMax(AlgaeConstants.grabberMotorId, MotorType.kBrushless); // makes new motor controller that is
        SparkMaxConfig config = new SparkMaxConfig();// defined as the motor for the arm
        SoftLimitConfig softLimit = new SoftLimitConfig();
        poseController = new PIDController(0.1, 0, 0);
        poseController.setTolerance(0.1);
        


        softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(43)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0);

        config
        //.inverted(true)
        .idleMode(IdleMode.kBrake)
       .apply(softLimit);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1.0, 0.0, 0.0, 0.0)
            .iZone(0)
            .outputRange(-1, 1);

            grabber.getEncoder().setPosition(0);
       grabber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public boolean HasNote() {
        return sensor.get();

    }

    public void grab() { // raises the roof

        grabber.set(-.30);
        SmartDashboard.putBoolean("Sensor Value", sensor.get());
        

    
    }

    // No more algae
    public void release() { // lowers the roof

        grabber.set(.15);
        

    }

    public void setSpeed(double i) { // lowers the roof

        grabber.set(i);
        

    }

    public void stop() { // stops the roof
        grabber.set(0);

    }
   
    public PIDController getController() {
        return poseController;
      }

    public void setPosition(double position) { // lowers the roof

        grabber.getEncoder().setPosition(position);
        

    }

    public double getposition() { // gets position
        return grabber.getEncoder().getPosition();
    }
    
    

}

