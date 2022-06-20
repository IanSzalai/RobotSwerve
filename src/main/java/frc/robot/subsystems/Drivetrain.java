// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import static frc.robot.RobotMap.map_Drivetrain.*;
import static frc.robot.RobotPreferences.pref_Drivetrain.*;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  TalonFXConfiguration driveConfig;
  TalonFXConfiguration turnConfig;

  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private final AHRS navX;

  private final SwerveDriveKinematics kinematics;

  private final SwerveDriveOdometry odometry;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeft = new SwerveModule(FRONT_LEFT_DRIVE, FRONT_LEFT_TURN,
        FRONT_LEFT_TURN_ENCODER);
    frontRight = new SwerveModule(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURN,
        FRONT_RIGHT_TURN_ENCODER);
    backLeft = new SwerveModule(BACK_LEFT_DRIVE, BACK_LEFT_TURN,
        BACK_LEFT_TURN_ENCODER);
    backRight = new SwerveModule(BACK_RIGHT_DRIVE, BACK_RIGHT_TURN,
        BACK_RIGHT_TURN_ENCODER);

    driveConfig = new TalonFXConfiguration();
    turnConfig = new TalonFXConfiguration();

    double d = moduleDistOut.getValue();

    frontLeftLocation = new Translation2d(d, d);
    frontRightLocation = new Translation2d(d, -d);
    backLeftLocation = new Translation2d(-d, d);
    backRightLocation = new Translation2d(-d, -d);

    navX = new AHRS();
    navX.reset();

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    odometry = new SwerveDriveOdometry(kinematics, navX.getRotation2d());

    configure();
  }

  public void configure() {
    frontLeft.configMotors(driveConfig, turnConfig);
    frontRight.configMotors(driveConfig, turnConfig);
    backLeft.configMotors(driveConfig, turnConfig);
    backRight.configMotors(driveConfig, turnConfig);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navX.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed.getValue());
    frontLeft.setDesiredState(moduleStates[0]);
    frontRight.setDesiredState(moduleStates[1]);
    backLeft.setDesiredState(moduleStates[2]);
    backRight.setDesiredState(moduleStates[3]);
  }

  public void openLoopDrive(double driveSpeed, double turnSpeed) {
    frontLeft.setDriveMotor(driveSpeed);
    frontRight.setDriveMotor(driveSpeed);
    backLeft.setDriveMotor(driveSpeed);
    backRight.setDriveMotor(driveSpeed);

    frontLeft.setTurnMotor(turnSpeed);
    frontRight.setTurnMotor(turnSpeed);
    backLeft.setTurnMotor(turnSpeed);
    backRight.setTurnMotor(turnSpeed);
  }

  private void updateOdometry() {
    odometry.update(
        navX.getRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    // SmartDashboard.putNumber("Front Left Drive Encoder Position", frontLeft.getDriveEncoderPosition());
    SmartDashboard.putNumber("Front Left Turn SRX Mag Encoder", frontLeft.getTurnEncoderPosition());

    // SmartDashboard.putNumber("Front Right Drive Encoder Position", frontRight.getDriveEncoderPosition());
    SmartDashboard.putNumber("Front Right Turn SRX Mag Encoder", frontRight.getDriveEncoderPosition());

    // SmartDashboard.putNumber("Back Left Drive Encoder Position", backLeft.getDriveEncoderPosition());
    SmartDashboard.putNumber("Back Left Turn SRX Mag Encoder", backLeft.getTurnEncoderPosition());

    // SmartDashboard.putNumber("Back Right Drive Encoder Position", backRight.getDriveEncoderPosition());
    SmartDashboard.putNumber("Back Right Turn SRX Mag Encoder", backRight.getDriveEncoderPosition());
    
    SmartDashboard.putNumber("Front Left Drive Falcon Integrated Encoder", frontLeft.getDriveIntegratedEncoder());
    SmartDashboard.putNumber("Front Left Turn Falcon Integrated Encoder", frontLeft.getTurnIntegratedEncoder());

    SmartDashboard.putNumber("Front Right Drive Falcon Integrated Encoder", frontRight.getDriveIntegratedEncoder());
    SmartDashboard.putNumber("Front Right Turn Falcon Integrated Encoder", frontRight.getTurnIntegratedEncoder());

    SmartDashboard.putNumber("Back Left Drive Falcon Integrated Encoder", backLeft.getDriveIntegratedEncoder());
    SmartDashboard.putNumber("Back Left Turn Falcon Integrated Encoder", backLeft.getTurnIntegratedEncoder());

    SmartDashboard.putNumber("Back Right Drive Falcon Integrated Encoder", backRight.getDriveIntegratedEncoder());
    SmartDashboard.putNumber("Back Right Turn Falcon Integrated Encoder", backRight.getTurnIntegratedEncoder());
  }
}
