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

public class CoralPivot extends SubsystemBase {
    


    private final SparkMax coralPivot;
    private DigitalInput sensor;
    SparkClosedLoopController pidController;

    private PIDController heightController;

    public CoralPivot() {
        coralPivot = new SparkMax(CoralConstants.coralPivotMotorId, MotorType.kBrushless); // makes new motor controller that is
        SparkMaxConfig config = new SparkMaxConfig();// defined as the motor for the arm

        heightController = new PIDController(0.1, 0, 0);
    heightController.setTolerance(0.1);

        config
        //.inverted(true)
        .idleMode(IdleMode.kBrake);
       coralPivot.getEncoder().setPosition(0);


        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1.0, 0.0, 0.0, 0.0)
            .iZone(0)
            .outputRange(-1, 1);

       coralPivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sensor = new DigitalInput(1);
    }

    public boolean HasNote() {
        return sensor.get();

    }

    public void raise() { // raises the roof

        coralPivot.set(-.30);
        SmartDashboard.putBoolean("Sensor Value", sensor.get());
        

    
    }

    // No more calamari
    public void lower() { // lowers the roof

        coralPivot.set(.15);
        

    }

    public void stop() { // stops the roof
        coralPivot.set(0);

    }

    public void setPosition(double position) { // lowers the roof

        coralPivot.getEncoder().setPosition(position);
        

    }

    public double getposition() { // gets position
        return coralPivot.getEncoder().getPosition();
    }
public double getHeight() {
  return coralPivot.getEncoder().getPosition();
}

public PIDController getController() {
  return heightController;
}

public void setSpeed(double i) {
    coralPivot.set(i);
}
}

