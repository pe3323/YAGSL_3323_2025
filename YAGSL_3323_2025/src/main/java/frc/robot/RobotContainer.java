// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import frc.robot.commands.AlgaeRetract;
import frc.robot.commands.SetCoralAngle;
import frc.robot.commands.SetCoralPivot;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetHarpoonAngle;
import frc.robot.commands.SetLockAngle;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.swervedrive.Coral;
import frc.robot.subsystems.swervedrive.CoralPivot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
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
  private final Algae algae = new Algae();
  private final CoralPivot coralPivot = new CoralPivot();
  private final AlgaeGrabber algaeGrabber = new AlgaeGrabber();
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

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * 1,
      () -> driverXbox.getLeftX() * 1)
      .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
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
    NamedCommands.registerCommand("coralDeposit", new SetCoralAngle(coral, 125)); 
    NamedCommands.registerCommand("coralDefault", new SetCoralAngle(coral, 0));
    NamedCommands.registerCommand("algaeExtend", new AlgaeExtend(algae, 2, lightsSubsystem));
    NamedCommands.registerCommand("algaeRetract", new AlgaeRetract(algae, 2, lightsSubsystem)); 
    NamedCommands.registerCommand("CoralPivot", new SetCoralPivot(coralPivot, 45));
    

    lightsSubsystem.setSolidColor(128,128, 0);


    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Add autonomous commands here  
    m_chooser.setDefaultOption("Do_Nothing", new PathPlannerAuto("C2"));
    m_chooser.addOption("Default", new PathPlannerAuto("Default"));
    m_chooser.addOption("LB1_1LS", new PathPlannerAuto("LB1_1LS"));
    m_chooser.addOption("RB3_3RS", new PathPlannerAuto("RB3_3RS"));
    m_chooser.addOption("LB1_1LS_2", new PathPlannerAuto("LB1_1LS_2"));
    m_chooser.addOption("RB3_3RS_2", new PathPlannerAuto("RB3_3RS_2"));
    m_chooser.addOption("RB3_RS3_3", new PathPlannerAuto("RB3_RS3_3"));
    m_chooser.addOption("LB1_LS1_3", new PathPlannerAuto("LB1_LS1_3"));
    m_chooser.addOption("RB3_3RS_RS4_4RS", new PathPlannerAuto("RB3_3RS_RS4_4RS"));
    m_chooser.addOption("RB3_3RS_RS4_4RS_2", new PathPlannerAuto("RB3_3RS_RS4_4RS_2"));
    m_chooser.addOption("LB1_1LS_LS6_6LS_LS5", new PathPlannerAuto("LB1_1LS_LS6_6LS_LS5"));
    m_chooser.addOption("LB1_1LS_2_LS6_6LS", new PathPlannerAuto("LB1_1LS_2_LS6_6LS"));


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
      driverXbox.start().onTrue(
        new SequentialCommandGroup(
          new SetCoralPivot(coralPivot, 45),
          new SetCoralAngle(coral, -45)
        )
      );
      /* driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none()); */
      


     



      //operatorXbox.a().onTrue( new  SetElevatorHeight(elevator, Constants.LEVEL0_HEIGHT) );
      //operatorXbox.x().onTrue( new  SetElevatorHeight(elevator, Constants.LEVEL1_HEIGHT) );
      //operatorXbox.b().onTrue( new  SetElevatorHeight(elevator, Constants.LEVEL2_HEIGHT) );
      //operatorXbox.y().onTrue( new  SetElevatorHeight(elevator, Constants.LEVEL3_HEIGHT) );

      operatorXbox.a().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, 0), new SetElevatorHeight(elevator, Constants.LEVEL0_HEIGHT), () -> elevator.atLevel0()));
      operatorXbox.x().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, 90), new SetElevatorHeight(elevator, Constants.LEVEL1_HEIGHT), () -> elevator.atLevel1()));
      operatorXbox.b().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, 90), new SetElevatorHeight(elevator, Constants.LEVEL2_HEIGHT), () -> elevator.atLevel2()));
      operatorXbox.y().onTrue( new  ConditionalCommand(new SetCoralAngle(coral, -35), new SetElevatorHeight(elevator, Constants.LEVEL3_HEIGHT), () -> elevator.atLevel3()));
      operatorXbox.start().onTrue(
        new SetElevatorHeight(elevator, Constants.LEVEL05_HEIGHT)
      );

      /*
      operatorXbox.a().onTrue( new Command() {

        Command toRun;
        @Override
        public void initialize() {
          if (!level0){
            toRun = new SetElevatorHeight(elevator, 0);
            level0 = true;
            level1 = false;
            level2 = false;
            level3 = false;
            SmartDashboard.putBoolean(getName(), level0);
            SmartDashboard.putBoolean(getName(), level1);
            SmartDashboard.putBoolean(getName(), level2);
            SmartDashboard.putBoolean(getName(), level3);
          } else {
             toRun = new SetCoralAngle(coral, 0);
             
          }
        }

        @Override
        public void execute() {
          toRun.execute();
        }

        @Override
        public boolean isFinished(){
          return toRun.isFinished();
        }
      });


      operatorXbox.x().onTrue( new Command() {


        Command toRun;
        @Override
        public void initialize() {
          System.out.println( "L1 inited");
          if (!level1){
            System.out.println( "Switching to elevator");

            toRun = new SetElevatorHeight(elevator, Constants.LEVEL1_HEIGHT, lightsSubsystem);
            level0 = false;
            level1 = true;
            level2 = false;
            level3 = false;
            SmartDashboard.putBoolean(getName(), level0);
            SmartDashboard.putBoolean(getName(), level1);
            SmartDashboard.putBoolean(getName(), level2);
            SmartDashboard.putBoolean(getName(), level3);
          } else {
            System.out.println( "Switching to coral");
             toRun = new SetCoralAngle(coral, 45);
          }
        }

        @Override
        public void execute() {
          System.out.println( "Executing Level 1");
          toRun.execute();
        }

        @Override
        public boolean isFinished(){
          return toRun.isFinished();
        }
      });
       

      operatorXbox.b().onTrue( new Command() {

        Command toRun;
        @Override
        public void initialize() {
          if (!level2){
            toRun = new SetElevatorHeight(elevator,Constants.LEVEL2_HEIGHT );
            level0 = false;
            level1 = false;
            level2 = true;
            level3 = false;
            SmartDashboard.putBoolean(getName(), level0);
            SmartDashboard.putBoolean(getName(), level1);
            SmartDashboard.putBoolean(getName(), level2);
            SmartDashboard.putBoolean(getName(), level3);
          } else {
             toRun = new SetCoralAngle(coral, 45);
          }
        }

        @Override
        public void execute() {
          toRun.execute();
        }

        @Override
        public boolean isFinished(){
          return toRun.isFinished();
        }
      });

      operatorXbox.y().onTrue( new Command() {

        Command toRun;
        @Override
        public void initialize() {
          if (!level3){
            toRun = new SetElevatorHeight(elevator, Constants.LEVEL3_HEIGHT);
            level0 = false;
            level1 = false;
            level2 = false;
            level3 = true;
            SmartDashboard.putBoolean(getName(), level0);
            SmartDashboard.putBoolean(getName(), level1);
            SmartDashboard.putBoolean(getName(), level2);
            SmartDashboard.putBoolean(getName(), level3);
          } else {
             toRun = new SetCoralAngle(coral, 0);

          }
        }

        @Override
        public void execute() {
          toRun.execute();
        }

        @Override
        public boolean isFinished(){
          return toRun.isFinished();
        }
      });
      */
        operatorXbox.leftBumper().whileTrue(new Command() {
          @Override
          public void execute() {
           algaeGrabber.grab(); 
          }
    
          @Override
          public void end(boolean interrupted) {
            algaeGrabber.stop();
          }

          @Override
          public boolean isFinished(){
            return false;
          }
        });

        operatorXbox.leftTrigger().whileTrue(new Command() {
          @Override
          public void execute() {
           algaeGrabber.release(); 
          }
    
          @Override
          public void end(boolean interrupted) {
            algaeGrabber.stop();
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
        driverXbox.a().onTrue(
        new SetLockAngle(climber, -18, lightsSubsystem)
        );
       
        driverXbox.b().onTrue(
        new SetLockAngle(climber, 0, lightsSubsystem)  
        );


    }
    operatorXbox.rightBumper().whileTrue(new Command() {
      @Override
      public void execute() {
        algae.retract();
      }


      @Override
      public void end(boolean interuppted){

        algae.stop();
      }

      @Override
      public boolean isFinished(){
        return false;
      }

     
    });

    operatorXbox.rightTrigger().whileTrue(new Command() {
      @Override
      public void execute() {
        algae.extend();
        SmartDashboard.putNumber("AlgaeRots", algae.getposition());
      }

      @Override
      public void end(boolean interuppted){

        algae.stop();
      }

      @Override
      public boolean isFinished(){
        return false;
      }

      
    });

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
