package frc.robot.subsystems;
    
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Lights;

public class Algae extends SubsystemBase {
          


    private final SparkMax pivot;
    SparkClosedLoopController pidControllerPivot;
    private final SparkMax arm;
    SparkClosedLoopController pidControllerArm;
    private final TalonFX grabber;
    SparkClosedLoopController pidControllerGrabber;
    private PIDController armController = new PIDController(0.05,0,0);
    private PIDController pivotController = new PIDController(0.1,0,0);
    private CommandXboxController controller;
    private boolean controlLock = false;

    public Algae(CommandXboxController controller) {

    grabber= new TalonFX(AlgaeConstants.grabberMotorId);
        arm = new SparkMax(AlgaeConstants.armMotorId, MotorType.kBrushless);
        pivot = new SparkMax(AlgaeConstants.pivotMotorID, MotorType.kBrushless);
        armController.setTolerance(0.1);
        pivotController.setTolerance(0.1); // makes new motor controller that is
        SparkMaxConfig armConfig = new SparkMaxConfig();// defined as the motor for the arm
   TalonFXConfigurator wheels = grabber.getConfigurator();
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        SoftLimitConfig softLimit = new SoftLimitConfig();
        SoftLimitConfig softLimitPivot = new SoftLimitConfig();


        this.controller = controller;

        

        arm.clearFaults();
        
        softLimitPivot
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(-25);

        softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeConstants.maxAlgaePosition)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(AlgaeConstants.miniumumAlgaePostition);
      

        pivotConfig
        .idleMode(IdleMode.kBrake)
        .apply(softLimitPivot);
        //To-Do:add a soft limit, needs to be tested

        armConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
       .apply(softLimit);

     
            arm.getEncoder().setPosition(0);
       arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
       pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }





    public void pivotChange(double speed) { // raises the roof
        pivot.set(speed);
    }
    
    public PIDController getPivotController() {
        return pivotController;
      }


      public double getPivotPosition() { // gets position
        return arm.getEncoder().getPosition();
    }

    public double getPivotAngle(){
        return ((360*pivot.getEncoder().getPosition())/AlgaeConstants.pivotGear);
    }





    public double getArmAngle(){
        return ((360*arm.getEncoder().getPosition())/AlgaeConstants.armGear);
    }




    @Override
    public void periodic(){
        
        if (controlLock != true){
        if(Math.abs(controller.getLeftY()) < 0.1){
             pivotChange(0.0);
        } else {
            pivotChange(controller.getLeftY()*.75);
        }}
    }   
  
    public void setLocked(boolean newStatus){
        controlLock = newStatus;
    }


    
    public void raise() { // raises the roof

        arm.set(.20);
    
    }

    // No more algae
    public void lower() { // lowers the roof

        arm.set(-.20);
        

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
        grabber.set(-.5);
    
    }

    // No more algae
    public void release() { // lowers the roof

        grabber.set(1);
        

    }

    public void stopGrabber() { // stops the roof
        double currentPosition = grabber.getPosition().getValueAsDouble();
        grabber.set(0);
        PositionVoltage holdPosition = new PositionVoltage(currentPosition);
        grabber.setControl(holdPosition);
    }

    public PIDController getController() {
        return armController;
      }
      
      public double getPose() {
        return arm.getEncoder().getPosition();
      }


      public void setArmSpeed(double i) { // lowers the roof

        arm.set(i);
        

    }


    public void setGrabberSpeed(double i) { // lowers the roof

        grabber.set(i);
}
}

