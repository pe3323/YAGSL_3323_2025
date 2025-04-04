package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;

public class ElectronicLightOrchestra extends Command {
     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
      private Lights lightsSubsystem;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ElectronicLightOrchestra(Lights lightsSubsystem) {
      this.lightsSubsystem = lightsSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lightsSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
