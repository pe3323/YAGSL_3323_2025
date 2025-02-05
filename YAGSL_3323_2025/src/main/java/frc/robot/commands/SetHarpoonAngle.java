// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.Coral;


/** An example command that uses an example subsystem. */
public class SetHarpoonAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Climber harpoon;
  private PIDController harpoonController;
  
  private double endPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetHarpoonAngle(Climber harpoon, double angle) {
    this.harpoon = harpoon;
    harpoonController = harpoon.getHarpoonController();
    endPosition = (angle/360);
    addRequirements(harpoon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    harpoonController.setSetpoint(endPosition);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    harpoon.setHarpoonSpeed(harpoonController.calculate(harpoon.getHarpoon()));
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    harpoon.stopHarpoon();
    harpoon.setHarpoonPosition(endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return harpoonController.atSetpoint();
  }
}
