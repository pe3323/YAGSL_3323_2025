// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.Elevator;


/** An example command that uses an example subsystem. */
public class AlgaeGrab extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private AlgaeGrabber grabber;
  private PIDController elevatorController;
  private double endPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeGrab(AlgaeGrabber grabber, double position) {
    this.grabber = grabber;
    elevatorController = grabber.getController();
    endPosition = position;
    addRequirements(grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorController.setSetpoint(endPosition);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabber.setSpeed(elevatorController.calculate(grabber.getposition()));
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabber.stop();
    grabber.setPosition(endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorController.atSetpoint();
  }
}
