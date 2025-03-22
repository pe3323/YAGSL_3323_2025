// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;


/** An example command that uses an example subsystem. */
public class SetElevatorHeight extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Elevator elevator;
  private PIDController elevatorController;
  private double endPosition;
  private Lights lightsSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetElevatorHeight(Elevator elevator, double position) {
    this.elevator = elevator;
    //this.lightsSubsystem = lightsSubsystem;
    elevatorController = elevator.getController();
    if (position ==  0)
     endPosition = 0;
    else
      endPosition = ElevatorConstants.gearRatio*((position)/ElevatorConstants.drumCircumferenceIn);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Target height: ", endPosition);
    elevatorController.setSetpoint(endPosition);

    if ( endPosition == ElevatorConstants.gearRatio*(Constants.LEVEL0_HEIGHT/ElevatorConstants.drumCircumferenceIn) ) {
      elevator.setLevel(0);
    }
    if ( endPosition == ElevatorConstants.gearRatio*(Constants.LEVEL05_HEIGHT/ElevatorConstants.drumCircumferenceIn) ) {
      elevator.setLevel(05);
    }
    if ( endPosition == ElevatorConstants.gearRatio*((Constants.LEVEL1_HEIGHT)/ElevatorConstants.drumCircumferenceIn) ) {
      elevator.setLevel(1);
    }
    if ( endPosition == ElevatorConstants.gearRatio*((Constants.LEVEL2_HEIGHT)/ElevatorConstants.drumCircumferenceIn) ) {
      elevator.setLevel(2);
    }
    if ( endPosition == ElevatorConstants.gearRatio*((Constants.LEVEL3_HEIGHT)/ElevatorConstants.drumCircumferenceIn) ) {
      elevator.setLevel(3);
    }    
    //lightsSubsystem.setSolidColor(19, 173, 14);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //lightsSubsystem.setSolidColor(19, 173, 14);

    double minDownPercentVoltage = -0.30;
    double maxDownPercentVoltage = 0.30;

    double speed = MathUtil.clamp(elevatorController.calculate(elevator.getHeight()), minDownPercentVoltage, maxDownPercentVoltage);
    SmartDashboard.putNumber("Current Height", elevator.getHeight());
    SmartDashboard.putNumber("Speed", speed);

    elevator.setSpeed(speed);
  }
    

  // Called once the command ends or is interrupted. 
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    elevator.setPosition(endPosition);
    SmartDashboard.putNumber("Current Height", elevator.getHeight());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("IsFinished", elevatorController.atSetpoint());
    //lightsSubsystem.setSolidColor(136, 9, 227);
    return elevatorController.atSetpoint();
  }
}
