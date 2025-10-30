package frc.robot.subsystems;
    
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeGrabber extends SubsystemBase {
          


    private final SparkMax grabber;
    SparkClosedLoopController pidController;
    private PIDController poseController;

    public AlgaeGrabber() {
        grabber = new SparkMax(AlgaeConstants.grabberMotorId, MotorType.kBrushless); // makes new motor controller that is
        SparkMaxConfig config = new SparkMaxConfig();// defined as the motor for the arm
     
        poseController = new PIDController(0.1, 0, 0);
        poseController.setTolerance(0.1);
        
        

        config
        //.inverted(true)
        .idleMode(IdleMode.kBrake);
       

        /*
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1.0, 0.0, 0.0, 0.0)
            .iZone(0)
            .outputRange(-1, 1);
            */

            grabber.getEncoder().setPosition(0);
       grabber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void grab() { // raises the roof
        grabber.set(-.3);
    
    }

    // No more algae
    public void release() { // lowers the roof

        grabber.set(.3);
        

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

