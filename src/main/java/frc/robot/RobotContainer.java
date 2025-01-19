// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.swerve.drivebase.TeleopDrive;
import frc.robot.Subsystems.Swerve.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverLeft = new CommandJoystick(Constants.OperatorConstants.DRIVER_LEFT);
  CommandJoystick driverRight = new CommandJoystick(Constants.OperatorConstants.DRIVER_RIGHT);
  CommandJoystick driverButtons = new CommandJoystick(Constants.OperatorConstants.DRIVER_BUTTONS);
  CommandJoystick coDriverLeft = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_LEFT);
  CommandJoystick coDriverRight = new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_RIGHT);
  CommandJoystick coDriverButtons =
      new CommandJoystick(Constants.OperatorConstants.CO_DRIVER_BUTTONS);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    TeleopDrive closedFieldRel =
        new TeleopDrive(
            () ->
                MathUtil.applyDeadband(
                    -driverLeft.getRawAxis(1), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -driverLeft.getRawAxis(0),
            () -> -driverRight.getRawAxis(0));
    Swerve.getInstance();
    Swerve.getInstance().setDefaultCommand(closedFieldRel);
    SmartDashboard.putString("test", "test");
  }
}
