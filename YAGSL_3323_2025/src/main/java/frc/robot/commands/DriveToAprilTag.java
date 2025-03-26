package frc.robot.commands;

import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

public class DriveToAprilTag extends Command{
    
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
  private Pose2d goalPose;

 

  private PIDController angleAngle = new PIDController(.1,0,0);

 // private PhotonCamera c;

  public DriveToAprilTag( SwerveSubsystem swerve) { // PhotonCamera c1 ) {
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

    goalPose = new Pose2d(14.268, 4.275 , Rotation2d.fromDegrees(robotTargetYaw));
  }

  @Override
  public void execute() {
    swerve.driveToPose(goalPose);
  
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
    //currentPose
       return false;
      
    

    
  }

}
