// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule extends SubsystemBase {
  private TalonFX driveMotor;
  private CANSparkMax turnMotor;

  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANCoder absoluteEncoder;

  private boolean absoluteEncoderReversed;
  private double absoluteEncoderOffset;
  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turnMotorId, boolean driveMotorReversed, boolean turnMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffset = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new CANCoder(absoluteEncoderId);

      driveMotor = new TalonFX(driveMotorId);
      turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      turnMotor.setInverted(turnMotorReversed);

      driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      turnEncoder = turnMotor.getEncoder();

      turnPIDController = new PIDController(Constants.SwerveConstants.KP_TURNING, 0, 0);
      turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

      resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBrake(boolean brake){
    if(brake){
      driveMotor.setNeutralMode(NeutralMode.Brake);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
    else{
      driveMotor.setNeutralMode(NeutralMode.Coast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getDriveMotorPosition(){
    return driveMotor.getSelectedSensorPosition() * Constants.SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity(){
    return driveMotor.getSelectedSensorVelocity() * Constants.SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition(){
    return turnEncoder.getPosition() * Constants.SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity(){
    return turnEncoder.getVelocity() * Constants.SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  public double getAbsoluteEncoderAngle(){
    double angle = absoluteEncoder.getAbsolutePosition();
    angle -= absoluteEncoderOffset;
    angle *= (Math.PI / 180);
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }

  public void resetEncoders(){
    driveMotor.setSelectedSensorPosition(0);
    turnEncoder.setPosition(getAbsoluteEncoderAngle() / Constants.SwerveConstants.TURN_MOTOR_PCONVERSION);
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
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / Constants.SwerveConstants.DRIVETRAIN_MAX_SPEED);
    turnMotor.set(turnPIDController.calculate(getTurnMotorPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + driveMotor.getDeviceID() + "] State", state.toString());
  }
  
  public void stop(){
    driveMotor.set(ControlMode.PercentOutput, 0);
    turnMotor.set(0);
  }
}
