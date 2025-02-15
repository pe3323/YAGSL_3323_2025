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
import frc.robot.subsystems.swervedrive.CoralPivot;


/** An example command that uses an example subsystem. */
public class SetCoralPivot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private CoralPivot coralPivot;
  private PIDController coralController;
  private double endPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetCoralPivot(CoralPivot coralPivot, double angle) {
    this.coralPivot = coralPivot;
    coralController = coralPivot.getController();
    endPosition = (angle/360) * CoralConstants.coralPivotGear;
    addRequirements(coralPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralController.setSetpoint(endPosition);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralPivot.setSpeed(coralController.calculate(coralPivot.getHeight()));
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralPivot.stop();
    coralPivot.setPosition(endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralController.atSetpoint();
  }
}
