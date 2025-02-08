package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.complex.CancelAllCommands;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ManuelControll.ElevatorManualControl;
import frc.robot.Commands.ManuelControll.WristManualControl;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.ReefSelection.SetTargetReefLevel;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.WristStates;
import frc.robot.Subsystems.Wrist.WristStates;

public class ButtonConfig {
  static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  static CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  static CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  static CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  static CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

  public void initTeleop() {

    driverButtons
        .button(4)
        .onTrue(
            new InstantCommand(
                () -> {
                  Swerve.getInstance().zeroGyro();
                }));
    // reef align

    coDriverLeft.button(1).onTrue(new SetOrientation(0));
    coDriverLeft.button(2).onTrue(new SetOrientation(1));

    coDriverButtons.button(7).onTrue(new ReefLevelSelection(2));
    coDriverButtons.button(9).onTrue(new ReefLevelSelection(1));
    coDriverButtons.button(11).onTrue(new ReefLevelSelection(0));

    driverButtons.button(5).onTrue(new SetCoralRollersState(CoralRollersState.INTAKING));
    driverButtons.button(6).onTrue(new SetCoralRollersState(CoralRollersState.OUTPUT));
    
    coDriverButtons.button(13).whileTrue(new ElevatorManualControl(() ->
    MathUtil.applyDeadband(
        -ButtonConfig.coDriverRight.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND)));

    coDriverButtons.button(13).whileTrue(new WristManualControl(() ->
    MathUtil.applyDeadband(
        -ButtonConfig.coDriverLeft.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND)));

    driverButtons.button(7)
        .onTrue(new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).andThen(new WaitCommand(1))
            .andThen(new SetElevatorState(ElevatorStates.LEVEL3)));

    driverButtons.button(8)
        .onTrue(new SetWristState(WristStates.VERTICAL, ClosedLoopSlot.kSlot0).andThen(new WaitCommand(1))
            .andThen(new SetElevatorState(ElevatorStates.LEVEL4)));
    driverButtons.button(9)
        .onTrue(new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).andThen(new WaitCommand(1))
            .andThen(new SetElevatorState(ElevatorStates.LEVEL2)));

    driverButtons.button(10)
        .onTrue(new SetCoralRollersState(CoralRollersState.OUTPUT)
            .andThen(new SetWristState(WristStates.BACKOF, ClosedLoopSlot.kSlot1))
            .andThen(new WaitCommand(0.5))
            .andThen(new SetCoralRollersState(CoralRollersState.STOPPED)));

    driverButtons.button(11).onTrue(new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0)
        .andThen(new SetElevatorState(ElevatorStates.LOW)));

    driverRight.button(1).whileTrue(new ScoreCoral(Elevator.getInstance().getReefTargetLevel(),
        Swerve.getInstance().getReefTargetSide(), Swerve.getInstance().getReefTargetOrientation()));
  }
}
