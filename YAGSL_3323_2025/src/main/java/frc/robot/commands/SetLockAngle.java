// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.swervedrive.Coral;


/** An example command that uses an example subsystem. */
public class SetLockAngle extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private Climber lock;
  private PIDController lockController;
  
  private double endPosition;

  private Lights lightsSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetLockAngle(Climber lock, double angle, Lights lightsSubsystem) {
    this.lock = lock;
    this.lightsSubsystem = lightsSubsystem;
    lockController = lock.getLockController();
    endPosition = (angle);
    addRequirements(lock);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockController.setSetpoint(endPosition);
    lightsSubsystem.setSolidColor(136, 9, 227);
  }
    

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lock.setLockSpeed(lockController.calculate(lock.getLock()));
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lock.stopLock();
    lock.setLockPosition(endPosition);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    lightsSubsystem.setSolidColor(255, 222, 33);   
    return lockController.atSetpoint();
    
  }
}
