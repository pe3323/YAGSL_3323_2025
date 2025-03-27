package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;


/** An example command that uses an example subsystem. */
public class ArmSetPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Algae algae;
  private PIDController armController;
  private PIDController pivotController;
  private double endPosition;
  private Lights lightsSubsystem;
  private String direction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmSetPosition(Algae algae, double angle, String direction) {
    this.algae = algae;
    this.direction = direction;
    //this.lightsSubsystem = lightsSubsystem;
    armController = algae.getController();
    pivotController = algae.getPivotController();
    if (angle ==  0)
     endPosition = 0;
    else
      endPosition = (angle/360) * AlgaeConstants.armGear;
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Target height:", endPosition);
    armController.setSetpoint(endPosition);
    algae.setLocked(true);

    algae.pivotChange(0);

    if (direction.equalsIgnoreCase("down")){
      pivotController.setSetpoint((-90/360) * AlgaeConstants.pivotGear);

    }
    else {
      pivotController.setSetpoint(0);
    }
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //lightsSubsystem.setSolidColor(19, 173, 14);

    double minDownPercentVoltage = -0.30;
    double maxDownPercentVoltage = 0.30;

    double speed = armController.calculate(algae.getposition());
    
    SmartDashboard.putNumber("Current Arm location", algae.getposition());
    SmartDashboard.putNumber("Speed", speed);

    algae.setArmSpeed(speed);

    if (direction.equalsIgnoreCase("down") && Math.abs(algae.getArmAngle()) > 30) {
      if (Math.abs(algae.getPivotAngle()) < 90){
        double pivotSpeed = MathUtil.clamp(pivotController.calculate(algae.getPivotPosition()), -.5, .5);
        algae.pivotChange(pivotSpeed);
        SmartDashboard.putNumber("pivot speed", pivotSpeed);
        SmartDashboard.putNumber("pivot angle", algae.getPivotAngle());
        SmartDashboard.putNumber("arm speed", speed);
        SmartDashboard.putNumber("arm angle", algae.getArmAngle());
      }
    }
    else if (direction.equalsIgnoreCase("up") && Math.abs(algae.getArmAngle()) < 60) {
      if (Math.abs(algae.getPivotAngle()) > 0){
        double pivotSpeed = MathUtil.clamp(pivotController.calculate(algae.getPivotPosition()), -.5, .5);
        algae.pivotChange(pivotSpeed);
        SmartDashboard.putNumber("pivot speed", pivotSpeed);
        SmartDashboard.putNumber("pivot angle", algae.getPivotAngle());
        SmartDashboard.putNumber("arm speed", speed);
        SmartDashboard.putNumber("arm angle", algae.getArmAngle());
      }

    }
  }
    

  // Called once the command ends or is interrupted. 
  @Override
  public void end(boolean interrupted) {
    algae.stop();
    algae.setPosition(endPosition);
    SmartDashboard.putNumber("Current Height", algae.getposition());
    algae.setLocked(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("IsFinished", armController.atSetpoint());
    //lightsSubsystem.setSolidColor(136, 9, 227);
    return armController.atSetpoint() && pivotController.atSetpoint();
  }
}

