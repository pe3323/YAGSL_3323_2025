package frc.robot.commands;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
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
  private double targetHeading = 0.0;
  private Cameras camera;
  private double robotYaw = 0.0;
  private double robotTargetYaw=0.0;
  private Pose2d currentPose;
  private Pose2d referencePose;
  private double theta= 0.0;
 // private PhotonCamera c;

  public FindApril( SwerveSubsystem swerve) { // PhotonCamera c1 ) {
    this.swerve = swerve;
    //this.c = c1;
    addRequirements(swerve);
  }



    @Override
  public void initialize() {
    referencePose=swerve.getApril(7).toPose2d();
    targetHeading= referencePose.getRotation().getRadians();

    currentPose= swerve.getPose();
    theta= Math.acos(((referencePose.getX()*currentPose.getX())+(referencePose.getY()*currentPose.getY()))
    /(Math.sqrt(Math.pow(referencePose.getX(), 2)+Math.pow(referencePose.getY(), 2))*(Math.sqrt(Math.pow(currentPose.getX(), 2)+Math.pow(currentPose.getY(), 2))
    
    )));
  }

  @Override
  public void execute() {
    robotYaw = swerve.getHeading().getRadians();

      if ((robotYaw-targetHeading)<0){
        swerve.drive(new Translation2d(0,0), 
                                1, true); }
      else {
        swerve.drive(new Translation2d(0,0), 
                                -1, true);
      }                               
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
    

   
        robotYaw = swerve.getHeading().getRadians();
        SmartDashboard.putString("April Tag Finder Finished", "Target found");
    
        SmartDashboard.putNumber("Robot Yaw", robotYaw);
        SmartDashboard.putNumber("robotTargetYaw", robotTargetYaw);
        return Math.abs(robotYaw-targetHeading) <= .1;
      
    

    
  }

}
