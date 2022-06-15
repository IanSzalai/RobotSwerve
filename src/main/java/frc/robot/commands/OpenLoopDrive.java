// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.frcteam3255.joystick.SN_DualActionStick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class OpenLoopDrive extends CommandBase {

  private Drivetrain drivetrain;
  private SN_DualActionStick controller;

  /** Creates a new OpenLoopDrive. */
  public OpenLoopDrive(Drivetrain a_drivetrain, SN_DualActionStick a_controller) {
    drivetrain = a_drivetrain;
    controller = a_controller;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.openLoopDrive(controller.getLeftStickY(), controller.getRightStickX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
