// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AlgaeRollers.AlgaeRollersManual;
import frc.robot.Commands.CoralRollers.CoralRollersManual;
import frc.robot.Commands.Elevator.ElevatorManual;
import frc.robot.Commands.Wrist.WristManual;
import frc.robot.Commands.swerve.drivebase.TeleopDrive;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private AlgaeRollers algaeRollers;
  private CoralRollers coralRollers;
  private Elevator elevator;
  private Wrist wrist;

  private AlgaeRollersManual algaeRollersManual;
  private CoralRollersManual coralRollersManuel;
  private ElevatorManual elevatorManel;
  private WristManual wristManuel;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    TeleopDrive closedFieldRel =
        new TeleopDrive(
            () ->
              MathUtil.applyDeadband(
                -ButtonConfig.driverLeft.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
            () -> 
              MathUtil.applyDeadband(
                -ButtonConfig.driverLeft.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
            () -> 
              MathUtil.applyDeadband(
                ButtonConfig.driverRight.getRawAxis(0), OperatorConstants.RIGHT_X_DEADBAND));
    Swerve.getInstance();
    Swerve.getInstance().setDefaultCommand(closedFieldRel);

    ButtonConfig buttons = new ButtonConfig();
    buttons.initTeleop();



    // this.algaeRollers = new AlgaeRollers();
    // this.coralRollers = new CoralRollers();
    // this.elevator = new Elevator();
    // this.wrist = new Wrist();

    // this.algaeRollersManual = new AlgaeRollersManual();
    // this.coralRollersManuel = new CoralRollersManual();
    // this.elevatorManel = new ElevatorManual();
    // this.wristManuel = new WristManual();
  }
}
