// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance();

  public static final XboxController controller = new XboxController(0);
  private final JoystickButton resetHeading_12 = new JoystickButton(controller, XboxController.Button.kB.value);
    // public static final Joystick joystick = new Joystick(0);
    // private final JoystickButton resetHeading_12 = new JoystickButton(joystick, 12);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new SwerveDrive());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    resetHeading_12.whenPressed(() -> drivetrain.zeroHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand(){
    // An ExampleCommand will run in autonomous
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.SwerveConstants.AUTO_DRIVE_MAX_SPEED, 
      Constants.SwerveConstants.AUTO_DRIVE_MAX_ACCELERATION)
      .setKinematics(Constants.SwerveConstants.DRIVE_KINEMATICS);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(-1,0),
        new Translation2d(-2,0),
        new Translation2d(-1,-0.5)
      ), 
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      config);

      Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(-1,0),
          new Translation2d(-2,0),
          new Translation2d(-1,-0.5)
        ), 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
        config);
        
      Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(0,-0.5),
          new Translation2d(-1,-0.5),
          new Translation2d(-1,0.5),
          new Translation2d(-2,0.5),
          new Translation2d(-2,-0.5),
          new Translation2d(-1,-0.5),
          new Translation2d(-1,0.5),
          new Translation2d(0,0.5)
        ), 
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
        config);

    // Path path = Filesystem.getDeployDirectory().toPath().resolve("output/" + "Test" + ".wpilib.json");
    // Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);

    PIDController frontController = new PIDController(Constants.SwerveConstants.AUTO_kP_FRONT, 0, 0);
    PIDController sideController = new PIDController(Constants.SwerveConstants.AUTO_kP_SIDE, 0, 0);
    ProfiledPIDController turnController = new ProfiledPIDController(
      Constants.SwerveConstants.AUTO_kP_TURN, 0, 0, Constants.SwerveConstants.AUTO_TURN_CONTROLLER_CONSTRAINTS);
    turnController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand command = new SwerveControllerCommand(
      trajectory,
      drivetrain::getPose,
      Constants.SwerveConstants.DRIVE_KINEMATICS,
      frontController,
      sideController,
      turnController,
      drivetrain::setModuleStates,
      drivetrain
    );

    SwerveControllerCommand command2 = new SwerveControllerCommand(
      trajectory2,
      drivetrain::getPose,
      Constants.SwerveConstants.DRIVE_KINEMATICS,
      frontController,
      sideController,
      turnController,
      drivetrain::setModuleStates,
      drivetrain
    );

    SwerveControllerCommand command3 = new SwerveControllerCommand(
      trajectory3,
      drivetrain::getPose,
      Constants.SwerveConstants.DRIVE_KINEMATICS,
      frontController,
      sideController,
      turnController,
      drivetrain::setModuleStates,
      drivetrain
    );

    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
      command,
      command2,
      command3,
      new InstantCommand(() -> drivetrain.stopModules())
    );
  }
}
