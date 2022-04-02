// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront = new SwerveModule(
    driveMotorId, 
    turnMotorId, 
    driveMotorReversed, 
    turnMotorReversed, 
    absoluteEncoderId, 
    absoluteEncoderOffset, 
    absoluteEncoderReversed);

  private SwerveModule leftBack = new SwerveModule(
    driveMotorId, 
    turnMotorId, 
    driveMotorReversed, 
    turnMotorReversed, 
    absoluteEncoderId, 
    absoluteEncoderOffset, 
    absoluteEncoderReversed);

  private SwerveModule rightFront = new SwerveModule(
    driveMotorId, 
    turnMotorId, 
    driveMotorReversed, 
    turnMotorReversed, 
    absoluteEncoderId, 
    absoluteEncoderOffset, 
    absoluteEncoderReversed);

  private SwerveModule rightBack = new SwerveModule(
    driveMotorId, 
    turnMotorId, 
    driveMotorReversed, 
    turnMotorReversed, 
    absoluteEncoderId, 
    absoluteEncoderOffset, 
    absoluteEncoderReversed);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();

  private static final Drivetrain drivetrain = new Drivetrain();

  public static Drivetrain getInstance(){
    return getInstance();
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain() {
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Angle", getHeading());
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(desiredStates[0]);
    rightFront.setDesiredState(desiredStates[1]);
    leftBack.setDesiredState(desiredStates[2]);
    rightBack.setDesiredState(desiredStates[3]);
  }
}
