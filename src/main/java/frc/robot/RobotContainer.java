// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.AlgaeRollers.AlgaeRollersManual;
import frc.robot.Commands.CoralRollers.CoralRollersManual;
import frc.robot.Commands.Elevator.ElevatorManual;
import frc.robot.Commands.Wrist.WristManual;
import frc.robot.Commands.swerve.drivebase.TeleopDrive;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollers;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersIO;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersIOSim;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersIOSystems;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersIO;
import frc.robot.Subsystems.CoralRollers.CoralRollersIOSim;
import frc.robot.Subsystems.CoralRollers.CoralRollersIOSystems;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveIO;
import frc.robot.Subsystems.Swerve.SwerveIOSystem;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristIO;
import frc.robot.Subsystems.Wrist.WristIOSim;
import frc.robot.Subsystems.Wrist.WristIOSparkMax;
import frc.robot.Subsystems.Wrist.WristStates;

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
  private Swerve swerve;

  private AlgaeRollersManual algaeRollersManual;
  private CoralRollersManual coralRollersManuel;
  private ElevatorManual elevatorManel;
  private WristManual wristManuel;

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
    switch (Constants.currentMode) {
      case REAL:
        AlgaeRollers.setInstance(new AlgaeRollersIOSystems());
        algaeRollers = AlgaeRollers.getInstance();

        CoralRollers.setInstance(new CoralRollersIOSystems());
        coralRollers = CoralRollers.getInstance();

        Elevator.setInstance(new ElevatorIOSparkMax());
        elevator = Elevator.getInstance();

        Wrist.setInstance(new WristIOSparkMax());
        wrist = Wrist.getInstance();

        Swerve.setInstance(new SwerveIOSystem());
        swerve = Swerve.getInstance();
        break;

      case SIM:
        AlgaeRollers.setInstance(new AlgaeRollersIOSim());
        algaeRollers = AlgaeRollers.getInstance();

        CoralRollers.setInstance(new CoralRollersIOSim());
        coralRollers = CoralRollers.getInstance();

        Elevator.setInstance(new ElevatorIOSim());
        elevator = Elevator.getInstance();

        Wrist.setInstance(new WristIOSim());
        wrist = Wrist.getInstance();

        Swerve.setInstance(new SwerveIOSystem());
        swerve = Swerve.getInstance();
        break;

      case REPLAY:
        AlgaeRollers.setInstance(new AlgaeRollersIO() {});
        algaeRollers = AlgaeRollers.getInstance();

        CoralRollers.setInstance(new CoralRollersIO() {});
        coralRollers = CoralRollers.getInstance();

        Elevator.setInstance(new ElevatorIO() {});
        elevator = Elevator.getInstance();

        Wrist.setInstance(new WristIO() {});
        wrist = Wrist.getInstance();

        Swerve.setInstance(new SwerveIO() {});
        swerve = Swerve.getInstance();
        break;
      default:
        break;
    }

    TeleopDrive closedFieldRel =
        new TeleopDrive(
            () ->
                MathUtil.applyDeadband(
                    -driverLeft.getRawAxis(1), Constants.OperatorConstants.LEFT_X_DEADBAND),
            () -> -driverLeft.getRawAxis(0),
            () -> driverRight.getRawAxis(0));
    Swerve.getInstance();
    Swerve.getInstance().setDefaultCommand(closedFieldRel);
    SmartDashboard.putString("test", "test");

    driverLeft
        .button(3)
        .onTrue(
            new InstantCommand(
                () -> {
                  SmartDashboard.putString("Command Triggered", "Command");
                  wrist.setState(WristStates.FOURTYFIVE);
                }));
  }
}
