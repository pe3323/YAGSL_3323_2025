package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;


/** An example command that uses an example subsystem. */
public class ArmSetPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Algae algae;
  private PIDController armController;
  private double endPosition;
  private Lights lightsSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmSetPosition(Algae algae, double position) {
    this.algae = algae;
    //this.lightsSubsystem = lightsSubsystem;
    armController = algae.getController();
    if (position ==  0)
     endPosition = 0;
    else
      endPosition = position;
    addRequirements(algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Target height: ", endPosition);
    armController.setSetpoint(endPosition);

   
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //lightsSubsystem.setSolidColor(19, 173, 14);

    double minDownPercentVoltage = -0.30;
    double maxDownPercentVoltage = 0.30;

    double speed = MathUtil.clamp(armController.calculate(algae.getPose()), minDownPercentVoltage, maxDownPercentVoltage);
    SmartDashboard.putNumber("Current Arm location", algae.getPose());
    SmartDashboard.putNumber("Speed", speed);

    algae.setArmSpeed(speed);
  }
    

  // Called once the command ends or is interrupted. 
  @Override
  public void end(boolean interrupted) {
    algae.stop();
    algae.setPosition(endPosition);
    SmartDashboard.putNumber("Current Height", algae.getPose());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("IsFinished", armController.atSetpoint());
    //lightsSubsystem.setSolidColor(136, 9, 227);
    return armController.atSetpoint();
  }
}

