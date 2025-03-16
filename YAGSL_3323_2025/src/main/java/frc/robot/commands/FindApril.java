package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

public class FindApril extends Command{
    
  private final SwerveSubsystem swerve;
  private boolean isRunning = false;
  private Timer timer = new Timer();  
  private double targetYaw = 0.0;
  private Cameras camera;
  private double robotYaw = 0.0;
  private double robotTargetYaw=0.0;
 // private PhotonCamera c;

  public FindApril( SwerveSubsystem swerve) { // PhotonCamera c1 ) {
    this.swerve = swerve;
    //this.c = c1;
    addRequirements(swerve);
  }



    @Override
  public void initialize() {
    SmartDashboard.putBoolean("result0ispresent", false);
    SmartDashboard.putNumber("Robot Init Yaw", swerve.getHeading().getDegrees());
    robotYaw = swerve.getHeading().getDegrees();
    this.camera = swerve.getVision().getCamera("center");
      Optional<PhotonPipelineResult> resultO = camera.getLatestResult();
    if (resultO.isPresent())
      {
        SmartDashboard.putBoolean("result0ispresent", true);
        var result = resultO.get();

        if (result.hasTargets())
        {
          targetYaw = result.getBestTarget()
          .getYaw();
          SmartDashboard.putNumber("Target Yaw", targetYaw);

          robotTargetYaw = robotYaw-targetYaw;
          
        }
      }

  }

  @Override
  public void execute() {
      //timer.start();


      //Optional<PhotonPipelineResult> resultO = camera.getBestResult();
     // if (resultO.isPresent())
     // {
      //  var result = resultO.get();
      //  if (result.hasTargets())
      //  {
      //    swerve.drive(swerve.getTargetSpeeds(0,
      //                          0,
      //                          Rotation2d.fromDegrees(targetYaw+20))); // Not sure if this will work, more math may be required.

      
      //PhotonPipelineResult pipe = c.getAllUnreadResults().get(0);
      //if ( pipe == null )
      //{
      //  System.out.println( "NOPE");
      //  return;
      //}
      //targetYaw = -pipe.getTargets().get(0).getYaw();

      
        swerve.drive(new Translation2d(0,0), 
                                Rotation2d.fromDegrees(targetYaw).getRadians(), true);                                
      //  }
      //}
      isRunning = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isRunning = false;  
    swerve.drive(new Translation2d(0,0),0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    

   
        robotYaw = swerve.getHeading().getDegrees();
        SmartDashboard.putString("April Tag Finder Finished", "Target found");
    
        SmartDashboard.putNumber("Robot Yaw", robotYaw);
        SmartDashboard.putNumber("robotTargetYaw", robotTargetYaw);
        return Math.abs(robotYaw-targetYaw) <= 1;
      
    

    
  }

}
