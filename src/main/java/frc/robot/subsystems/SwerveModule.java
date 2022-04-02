// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase {
  private CANSparkMax driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private AnalogInput absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;
  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new AnalogInput(absoluteEncoderId);

      driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
      turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      turnMotor.setInverted(turnMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turnEncoder = turnMotor.getEncoder();

      driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.DRIVE_MOTOR_PCONVERSION);
      driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.DRIVE_MOTOR_VCONVERSION);
      turnEncoder.setPositionConversionFactor(Constants.SwerveConstants.TURN_MOTOR_PCONVERSION);
      turnEncoder.setVelocityConversionFactor(Constants.SwerveConstants.TURN_MOTOR_VCONVERSION);

      turnPIDController = new PIDController(1, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getDriveMotorPosition(){
    return driveEncoder.getPosition();
  }

  public double getDriveMotorVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition();
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity();
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2 * Math.PI;
    angle -= absoluteEncoderOffset;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle());
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] State", state.toString());
  }
  
  public void stop(){
    driveMotor.set(0);
    turnMotor.set(0);
  }
}
