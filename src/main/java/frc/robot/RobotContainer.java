// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_DualActionStick;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap.*;
import frc.robot.commands.Drive;
import frc.robot.commands.OpenLoopDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private static final SN_DualActionStick con_driver = new SN_DualActionStick(map_Controllers.DRIVER);
  private static final Drivetrain sub_drivetrain = new Drivetrain();
  private static final Drive com_drive = new Drive(sub_drivetrain, con_driver);
  private static final OpenLoopDrive com_openLoopDrive = new OpenLoopDrive(sub_drivetrain, con_driver);

  public RobotContainer() {
    configureButtonBindings();
    sub_drivetrain.setDefaultCommand(com_openLoopDrive);
  }

  private void configureButtonBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
