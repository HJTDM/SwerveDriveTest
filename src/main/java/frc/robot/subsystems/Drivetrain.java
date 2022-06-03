// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private SwerveModule leftFront = new SwerveModule(
    1, 
    5, 
    true, 
    false, 
    11, 
    212.7, 
    false);

  private SwerveModule leftBack = new SwerveModule(
    3, 
    7, 
    true, 
    false, 
    13, 
    54.58, 
    false);

  private SwerveModule rightFront = new SwerveModule(
    2, 
    6, 
    false, 
    false, 
    12, 
    318.922, 
    false);

  private SwerveModule rightBack = new SwerveModule(
    4, 
    8, 
    false, 
    false, 
    14, 
    197.809, 
    false);

  private ADIS16470_IMU gyro = new ADIS16470_IMU();
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(0));

  private static final Drivetrain drivetrain = new Drivetrain();

  public static Drivetrain getInstance(){
    return drivetrain;
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
    odometry.update(getHeadingRotation2d(), leftFront.getState(), rightFront.getState(), leftBack.getState(), rightBack.getState());
    SmartDashboard.putNumber("Robot Angle", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
  }
  
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, getHeadingRotation2d());
  }

  public void setAllMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(-getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setState(moduleStates[0]);
    rightFront.setState(moduleStates[1]);
    leftBack.setState(moduleStates[2]);
    rightBack.setState(moduleStates[3]);
  }
}
