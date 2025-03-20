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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Lights;

public class Algae extends SubsystemBase {
          


    private final SparkMax pivot;
    SparkClosedLoopController pidControllerPivot;
    private final SparkMax arm;
    SparkClosedLoopController pidControllerArm;
    private final SparkMax grabber;
    SparkClosedLoopController pidControllerGrabber;
    private PIDController poseController;
    private CommandXboxController controller;

    public Algae(CommandXboxController controller) {
        grabber = new SparkMax(AlgaeConstants.grabberMotorId, MotorType.kBrushless);
        arm = new SparkMax(AlgaeConstants.armMotorId, MotorType.kBrushless);
        pivot = new SparkMax(AlgaeConstants.pivotMotorID, MotorType.kBrushless); // makes new motor controller that is
        SparkMaxConfig armConfig = new SparkMaxConfig();// defined as the motor for the arm
        SparkMaxConfig wheels = new SparkMaxConfig();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SoftLimitConfig softLimit = new SoftLimitConfig();

        this.controller = controller;


        arm.clearFaults();
        
        softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeConstants.maxAlgaePosition)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(AlgaeConstants.miniumumAlgaePostition);
       
        wheels
        .idleMode(IdleMode.kBrake);

        pivotConfig
        .idleMode(IdleMode.kBrake);
        //To-Do:add a soft limit, needs to be tested

        armConfig
        //.inverted(true)
        .idleMode(IdleMode.kBrake)
       .apply(softLimit);

        armConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1.0, 0.0, 0.0, 0.0)
            .iZone(0)
            .outputRange(-1, 1);

            arm.getEncoder().setPosition(0);
       arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       grabber.configure(wheels, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void pivotChange(double speed) { // raises the roof

        pivot.set(speed);
    
    }
    
    @Override
    public void periodic(){
        pivotChange(controller.getLeftY());
    }   
  
    
    
    public void extend() { // raises the roof

        arm.set(.50);
    
    }

    // No more algae
    public void retract() { // lowers the roof

        arm.set(-.50);
        

    }

    public void stop() { // stops the roof
        arm.set(0);

    }

    public void setPosition(double position) { // lowers the roof

        arm.getEncoder().setPosition(position);
        

    }

    public double getposition() { // gets position
        return arm.getEncoder().getPosition();
    }
    
    public void grab() { // raises the roof
        grabber.set(-.3);
    
    }

    // No more algae
    public void release() { // lowers the roof

        grabber.set(.3);
        

    }

    public void stopGrabber() { // stops the roof
        grabber.set(0);

    }

    public PIDController getController() {
        return poseController;
      }
      public void setGrabberSpeed(double i) { // lowers the roof

        grabber.set(i);
        

    }
}

