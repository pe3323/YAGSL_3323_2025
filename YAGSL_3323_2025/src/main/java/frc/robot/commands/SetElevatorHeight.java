// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;


/** An example command that uses an example subsystem. */
public class SetElevatorHeight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Elevator elevator;
  private PIDController elevatorController;
  private double endPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetElevatorHeight(Elevator elevator, double position) {
    this.elevator = elevator;
    elevatorController = elevator.getController();
    endPosition = ElevatorConstants.gearRatio*(position/ElevatorConstants.drumCircumferenceIn)+ElevatorConstants.robotHeight;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorController.setSetpoint(endPosition);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(elevatorController.calculate(elevator.getHeight()));
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    elevator.setPosition(endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorController.atSetpoint();
  }
}
