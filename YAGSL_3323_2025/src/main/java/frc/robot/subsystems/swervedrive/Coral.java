package frc.robot.subsystems.swervedrive;
    
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
    


    private final SparkMax coral;
    private DigitalInput sensor;
    SparkClosedLoopController pidController;

    public Coral() {
        coral = new SparkMax(CoralConstants.coralMotorId, MotorType.kBrushless); // makes new motor controller that is
        SparkMaxConfig config = new SparkMaxConfig();// defined as the motor for the arm

        config
        //.inverted(true)
        .idleMode(IdleMode.kBrake);
       coral.getEncoder().setPosition(0);


        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(1.0, 0.0, 0.0, 0.0)
            .iZone(0)
            .outputRange(-1, 1);

       coral.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sensor = new DigitalInput(1);
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

    public double getposition() { // gets position
        return coral.getEncoder().getPosition();
    }

}

