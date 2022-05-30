// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */

  private Drivetrain drivetrain = RobotContainer.drivetrain;
  private double frontSpeed, sideSpeed, turnSpeed;
  private boolean fieldOriented;
  private SlewRateLimiter frontLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter sideLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
  private SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // xSpeed = -RobotContainer.joystick.getRawAxis(1);
    // ySpeed = RobotContainer.joystick.getRawAxis(0);
    // turnSpeed = RobotContainer.joystick.getRawAxis(2);
    // fieldOriented = !RobotContainer.joystick.getRawButton(3);

    frontSpeed = -RobotContainer.controller.getLeftY();
    sideSpeed = RobotContainer.controller.getLeftX();
    turnSpeed = RobotContainer.controller.getRightX();
    fieldOriented = !RobotContainer.controller.getRawButton(XboxController.Button.kA.value);

    SmartDashboard.putNumber("X Speed", frontSpeed);
    SmartDashboard.putNumber("Y Speed", sideSpeed);
    SmartDashboard.putNumber("Turn Speed", turnSpeed);

    frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
    sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
    turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;

    frontSpeed = frontLimiter.calculate(frontSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, drivetrain.getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    drivetrain.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
