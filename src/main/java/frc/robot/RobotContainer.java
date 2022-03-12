package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BallManipulatorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LifterConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoDriveCmd;
import frc.robot.commands.BallManipulatorAuto;
import frc.robot.commands.BallManipulatorCmd;
import frc.robot.commands.LifterPositionCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.BallManipulatorSubsystem;
import frc.robot.subsystems.LifterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final BallManipulatorSubsystem ballManipulatorSubsystem = new BallManipulatorSubsystem();
  private final LifterSubsystem lifterSubsystem = new LifterSubsystem();
  
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  
  private BallManipulatorCmd ballManipulatorCmd = new BallManipulatorCmd(ballManipulatorSubsystem, 
  () -> driverJoystick.getRawButtonPressed(BallManipulatorConstants.kIntakeFlipperToggleButtonPort),
  () -> driverJoystick.getRawButtonPressed(BallManipulatorConstants.kBucketToggleButtonPort),
  () -> driverJoystick.getRawButton(BallManipulatorConstants.kTopIntakeButtonPort),
  () -> driverJoystick.getRawButton(BallManipulatorConstants.kBottomIntakeButtonPort),
  () -> driverJoystick.getRawAxis(BallManipulatorConstants.kTopSliderAxis),
  () -> driverJoystick.getRawAxis(BallManipulatorConstants.kBottomSliderAxis));

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoystick.getRawButtonPressed(OIConstants.kDriverFieldOrientedButtonPort)));

    lifterSubsystem.setDefaultCommand(new LifterPositionCmd(lifterSubsystem,  
                () -> driverJoystick.getRawButtonPressed(LifterConstants.kLifterToggleButtonPort)));

    
    ballManipulatorSubsystem.setDefaultCommand(ballManipulatorCmd);
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 12).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  public Command getAutonomousCommand() {
    /*
    //1. create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);


    //important part
    //2. generate trajectory
    Trajectory turnTrajectory = TrajectoryGenerator.generateTrajectory(       //turnto face the first ball
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
            new Translation2d(0,0)
        ), 
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)), 
        trajectoryConfig
    );
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(       //go to the frist ball behind the robot 
        new Pose2d(0, 0, new Rotation2d(90)), 
        List.of(
            new Translation2d(-0.5,0)
        ), 
        new Pose2d(1, 0, Rotation2d.fromDegrees(90)), 
        trajectoryConfig
    );

    Trajectory trajectoryTwo = TrajectoryGenerator.generateTrajectory(    //go to the hub
      new Pose2d(1, 0, new Rotation2d(90)), 
      List.of(
          new Translation2d(-1,0)
      ), 
      new Pose2d(-1, -0.2, Rotation2d.fromDegrees(200)), 
      trajectoryConfig
    );

    Trajectory trajectorySimple = TrajectoryGenerator.generateTrajectory(    //go to the hub
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
          new Translation2d(0,-1)
      ), 
      new Pose2d(0, -2, Rotation2d.fromDegrees(0)), 
      trajectoryConfig
    );
    //end of improtant part 


    //3. define pidcontrollers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //4. construct command to follow trajectory
    SwerveControllerCommand turnSwerveControllerCommand = new SwerveControllerCommand(turnTrajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
    SwerveControllerCommand swerveControllerCommandTwo = new SwerveControllerCommand(trajectoryTwo, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
    SwerveControllerCommand swerveControllerCommandSimple = new SwerveControllerCommand(trajectorySimple, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);
*/
    //5. add some init and wrap up and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.zeroHeading()),
      new BallManipulatorAuto(ballManipulatorSubsystem),
      new AutoDriveCmd(swerveSubsystem, 3, 0, 0)
    );
  }

}
