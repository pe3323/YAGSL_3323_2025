// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.Coral;


/** An example command that uses an example subsystem. */
public class SetCoralAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Coral coral;
  private PIDController coralController;
  private double endPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetCoralAngle(Coral coral, double angle) {
    this.coral = coral;
    coralController = coral.getController();
    endPosition = (angle/360) * CoralConstants.coralGear;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralController.setSetpoint(endPosition);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral.setSpeed(coralController.calculate(coral.getHeight()));
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stop();
    coral.setPosition(endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralController.atSetpoint();
  }
}
