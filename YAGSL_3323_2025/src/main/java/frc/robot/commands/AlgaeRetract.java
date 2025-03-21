// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Lights;

/** An example command that uses an example subsystem. */
public class AlgaeRetract extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Algae algae;
  private Timer timer = new Timer();
  private double time;
  private final Lights lightsSubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeRetract(Algae algae, double time, Lights lightsSubsystem) {
    this.algae = algae;
    this.lightsSubsystem = lightsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algae, lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.lower();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.stop();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Orange
     if (algae.getposition() == AlgaeConstants.miniumumAlgaePostition) {
      lightsSubsystem.setSolidColor(255, 113, 31);
     }
    return timer.hasElapsed(time);
  }
}
