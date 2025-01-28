package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.complex.CancelAllCommands;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Commands.complex.SetTargetReefLevel;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class ButtonConfig {
  static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

  public void initTeleop() {

    driverButtons
        .button(4)
        .onTrue(
            new InstantCommand(
                () -> {
                  Swerve.getInstance().zeroGyro();
                }));
    // reef align

    // source align
    //driverButtons.button(1).whileTrue(new AlignSource());

    // climb(no commands yet)
    // driverButtons
    //     .button(16)
    //     .onTrue(); (change for switch)

    // processor align? (4)
    //driverButtons.button(4).whileTrue(new AlignSource());

    //driverButtons.button(-1).whileTrue(new AlignReef().andThen(new SetCoralRollersState(CoralRollersState.OUTPUT)));

    // driverButtons.button(-1).whileTrue(new ScoreCoral()); // TODO - Implement
    // NOTE - This is just for testing:
    driverButtons.button(5).onTrue(new SetCoralRollersState(CoralRollersState.INTAKING));
    driverButtons.button(6).onTrue(new SetCoralRollersState(CoralRollersState.OUTPUT));
    driverButtons.button(7).onTrue(new SetElevatorState(ElevatorStates.LEVEL1));
    driverButtons.button(8).onTrue(new SetElevatorState(ElevatorStates.LEVEL2));
    driverButtons.button(9).onTrue(new SetElevatorState(ElevatorStates.LEVEL3));
    driverButtons.button(10).onTrue(new SetElevatorState(ElevatorStates.LEVEL4));
  }
}
