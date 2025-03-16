package frc.robot.commands.runnables;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision.Cameras;

public class AprilTagAssist implements Runnable {

    private SwerveSubsystem swerve;
    
    
    public AprilTagAssist( SwerveSubsystem swerve ) {
            this.swerve = swerve;
    }


    @Override
    public void run() {
      Cameras camera = swerve.getVision().getCamera("center");
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      var result = resultO.get();
      double targetYaw = -result.getBestTarget().getYaw();

      swerve.drive(new Translation2d(0,0), 
                                Rotation2d.fromDegrees(targetYaw).getRadians(), true);
    }

    
}
