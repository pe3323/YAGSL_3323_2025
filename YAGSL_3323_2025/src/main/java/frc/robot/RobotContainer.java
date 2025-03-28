// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeExtend;
import frc.robot.commands.AlgaeGrab;
import frc.robot.commands.AlgaeRetract;
import frc.robot.commands.ArmSetPosition;
import frc.robot.commands.FindApril;
import frc.robot.commands.SetCoralAngle;
import frc.robot.commands.SetCoralPivot;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetHarpoonAngle;
import frc.robot.commands.SetLockAngle;
import frc.robot.commands.runnables.AprilTagAssist;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.swervedrive.Coral;
import frc.robot.subsystems.swervedrive.CoralPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;


import java.io.File;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Elevator elevator = new Elevator();
  private final Coral coral = new Coral();
  private final Climber climber = new Climber();
  
  //private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();
  
  private boolean level0 = true;
  private boolean level1 = false;
  private boolean level2 = false;
  private boolean level3 = false;
  private final Lights lightsSubsystem = new Lights(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...  
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));
  private final Algae algae = new Algae(operatorXbox);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 1,
      () -> driverXbox.getLeftX() * 1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() /2.0 * -1.0)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
        () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) *(Math.PI * 2),
        () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) *(Math.PI * 2)
      )
      .headingWhile(true);

      SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings

    

    NamedCommands.registerCommand("Level0", new SetElevatorHeight(elevator, 0));  
    NamedCommands.registerCommand("Level1", new SetElevatorHeight(elevator, Constants.LEVEL1_HEIGHT)); 
    NamedCommands.registerCommand("Level2", new SetElevatorHeight(elevator, Constants.LEVEL2_HEIGHT)); 
    NamedCommands.registerCommand("Level3", new SetElevatorHeight(elevator, Constants.LEVEL3_HEIGHT)); 
    NamedCommands.registerCommand("grabAlgae", new AlgaeGrab(algae, 2));
    NamedCommands.registerCommand("releaseAlgae", new AlgaeGrab(algae, -2));
    NamedCommands.registerCommand("algaeExtend", new AlgaeExtend(algae, 2, lightsSubsystem));
    NamedCommands.registerCommand("algaeRetract", new AlgaeRetract(algae, 2, lightsSubsystem)); 
    

    lightsSubsystem.setSolidColor(128,128, 0);


    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Add autonomous commands here  
    m_chooser.setDefaultOption("Do_Nothing", new PathPlannerAuto("Do_Nothing"));
    m_chooser.addOption("Mid", new PathPlannerAuto("C2"));
    m_chooser.addOption("Right", new PathPlannerAuto("Right"));
    m_chooser.addOption("Left", new PathPlannerAuto("Left"));
    



    SmartDashboard.putData("Autonomous Mode", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      operatorXbox.a().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, 0), new SetElevatorHeight(elevator, Constants.LEVEL0_HEIGHT), () -> elevator.atLevel0()));
      operatorXbox.x().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, 90), new SetElevatorHeight(elevator, Constants.LEVEL1_HEIGHT), () -> elevator.atLevel1()));
      operatorXbox.b().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, 90), new SetElevatorHeight(elevator, Constants.LEVEL2_HEIGHT), () -> elevator.atLevel2()));
      operatorXbox.y().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, -35), new SetElevatorHeight(elevator, Constants.LEVEL3_HEIGHT), () -> elevator.atLevel3()));
      operatorXbox.start().onTrue(
        new SetElevatorHeight(elevator, Constants.LEVEL05_HEIGHT)
      );


         operatorXbox.leftBumper().whileTrue(new Command() {
          @Override
          public void execute() {
           algae.grab(); 
          }  
    
          @Override
          public void end(boolean interrupted) {
            algae.stopGrabber();
          }

          @Override
          public boolean isFinished(){
            return false;
          }
        });

        operatorXbox.leftTrigger().whileTrue(new Command() {
          @Override
          public void execute() {
           algae.release(); 
          }
    
          @Override
          public void end(boolean interrupted) {
            algae.stopGrabber();
          }

          @Override
          public boolean isFinished(){
            return false;
          }
        });


        // end game - what follows are commands that  handle the
        // end of teleop mode.
        //driverXbox.leftBumper().onTrue(new ConditionalCommand())


        // Extend the harpoon and then pull us in.
        //driverXbox.start().onTrue((drivebase.driveToPose(new Pose2d(14.268, 4.275 , Rotation2d.fromDegrees(174.279)))));

        driverXbox.start().onTrue( Commands.runOnce(() -> {
            AprilTag t = Utils.getClosestReefAprilTag(drivebase.getPose());
            if ( t == null )
              SmartDashboard.putString("Goto Id", "None");
            else 
              SmartDashboard.putString("I Want to go to ID " , "" + t.ID);
        }, drivebase));

        //driverXbox.x().onTrue(
          //Commands.runOnce(new AprilTagAssist(drivebase), drivebase)
            //drivebase.aimAtTarget(drivebase.getVision().getCamera("center"))
       //     new FindApril(drivebase)
       // );



        driverXbox.leftBumper().whileTrue(new Command() {
          @Override
          public void execute() {
            climber.HarpoonRetract();
          }
    
          @Override
          public void end(boolean interrupted) {
            climber.stopHarpoon();
          };

          @Override
          public boolean isFinished(){
            return false;
          }
        });


        driverXbox.rightBumper().whileTrue(new Command() {
          @Override
          public void execute() {
            climber.HarpoonExtend();
            SmartDashboard.putNumber("HarpoonRots", climber.getHarpoon());
          } 
    
          @Override
          public void end(boolean interrupted) {
            climber.stopHarpoon();
          };

          @Override
          public boolean isFinished(){
            return false;
          }
        });


        // These commands handle the foot/hook/anchor thing
        // was -18  (49:1), then multiple 1.9 
        driverXbox.a().onTrue(
        new SetLockAngle(climber, -1.5, lightsSubsystem)
        );
       
    //RightTrigger pushes the algae arm down
    }
    operatorXbox.rightTrigger().onTrue(
     
        new ArmSetPosition(algae, -90)

    );

     
    
    //Right bumper sets the algae arm back to 0
    operatorXbox.rightBumper().onTrue(
        new ArmSetPosition(algae, 0)

    );
    
  }
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
