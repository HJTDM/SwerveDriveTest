// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
  /** Creates a new SwerveDrive. */

  private Drivetrain drivetrain = RobotContainer.drivetrain;
  private Supplier<Double> xSpdFunction, ySpdFunction, turnSpdFunction;
  private Supplier<Boolean> fieldOrientedFunction;
  private SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  public SwerveDrive(Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turnSpdFunction,
          Supplier<Boolean> fieldOrientedFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turnSpdFunction = turnSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    xLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    yLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    turnLimiter = new SlewRateLimiter(Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turnSpeed = turnSpdFunction.get();

    xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0;
    turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;

    xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, drivetrain.getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
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
