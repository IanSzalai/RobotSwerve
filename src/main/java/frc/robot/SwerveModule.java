// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import static frc.robot.RobotPreferences.pref_SwerveModule.*;

/** Swerve Module Class */
public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX turnMotor;

    private AnalogPotentiometer turnAbsEncoder;

    private final PIDController drivePIDController = new PIDController(
            drivePIDControllerP.getValue(),
            drivePIDControllerI.getValue(),
            drivePIDControllerD.getValue());

    private final ProfiledPIDController turnPIDController = new ProfiledPIDController(
            turnPIDControllerP.getValue(),
            turnPIDControllerI.getValue(),
            turnPIDControllerD.getValue(),
            new TrapezoidProfile.Constraints(
                    maxAngularVelocity.getValue(),
                    maxAngularAcceleration.getValue()));

    // Gains are for example purposes only - must be determined for your own robot!
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            driveFeedforwardS.getValue(), driveFeedforwardV.getValue());
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
            turnFeedforwardS.getValue(), turnFeedforwardV.getValue());

    //
    // From SDS MK4 Product Page
    // L1 - Standard: 8.14 : 1
    // L2 - Fast: 6.75 : 1
    // L3 - Very Fast: 6.12 : 1
    // L4 - Too Fast: 5.14 : 1
    //

    static final double gearingL1 = 8.14;
    static final double gearingL2 = 6.75;
    static final double gearingL3 = 6.12;
    static final double gearingL4 = 5.14;

    /**
     * Creates Swerve Module with Drive Encoder
     * 
     * @param driveMotorID       CAN device number of driveMotor
     * @param turnMotorID        CAN device number of turnMotor
     * @param turnEncoderPort    Analog Port number that the encoder is on 
     */
    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderPort) {

        driveMotor = new TalonFX(driveMotorID);
        turnMotor = new TalonFX(turnMotorID);

        turnAbsEncoder = new AnalogPotentiometer(turnEncoderPort);
    }

    public void configMotors(TalonFXConfiguration driveConfig, TalonFXConfiguration turnConfig) {
        driveMotor.configFactoryDefault();
        turnMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(false);
        turnMotor.setInverted(false);
        driveMotor.configAllSettings(driveConfig);
        turnMotor.configAllSettings(turnConfig);
    }

    public double getDriveEncoderPosition() {

        return driveMotor.getSelectedSensorPosition()
                * (2 * Math.PI * wheelRadius.getValue() / encoderCountsPerRotation.getValue())
                * gearingL2;
    }

    /**
     * Get the rate of the drive encoder
     * 
     * @return rate of drive encoder in meters per second
     */
    private double getDriveEncoderRate() {

        double rate = driveMotor.getSelectedSensorVelocity(); // sensor units per 100ms

        rate *= 10; // sensor units per second

        rate /= encoderCountsPerRotation.getValue(); // motor rotations per second

        rate *= gearingL2; // wheel rotations per second

        rate *= (2 * Math.PI * wheelRadius.getValue()); // meters per second

        return rate;
    }

    public double getTurnEncoderPosition() {
        return turnAbsEncoder.get();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveEncoderRate(), new Rotation2d(getTurnEncoderPosition()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        // eg at 359 degrees instead of traveling 360 degrees to get to 0, just travel 1
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoderPosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDController.calculate(getDriveEncoderRate(), state.speedMetersPerSecond);
        final double driveOutputFeedforward = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turnPIDController.calculate(getTurnEncoderPosition(), state.angle.getRadians());
        final double turnOutputFeedforward = turnFeedforward.calculate(turnPIDController.getSetpoint().velocity);

        driveMotor.set(ControlMode.Current, driveOutput + driveOutputFeedforward);
        turnMotor.set(ControlMode.Current, turnOutput + turnOutputFeedforward);
    }

    public void setDriveMotor(double percentOutput) {
        double demand = percentOutput * RobotPreferences.pref_SwerveModule.driveSpeedMultiplier.getValue();
        driveMotor.set(ControlMode.PercentOutput, demand);
    }

    public void setTurnMotor(double percentOutput) {
        double demand = percentOutput * RobotPreferences.pref_SwerveModule.turnSpeedMultiplier.getValue();
        turnMotor.set(ControlMode.PercentOutput, demand);
    }

    public double getDriveIntegratedEncoder() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getTurnIntegratedEncoder() {
        return turnMotor.getSelectedSensorPosition();
    }
}
