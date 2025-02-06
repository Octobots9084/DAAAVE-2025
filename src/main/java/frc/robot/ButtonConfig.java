package frc.robot;


import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.Wrist.SetWristState;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Wrist.WristStates;

public class ButtonConfig {
  static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

  public void initTeleop() {

    driverButtons
        .button(6)
        .onTrue(
            new InstantCommand(
                () -> {
                  Swerve.getInstance().zeroGyro();
                }));
    // reef align

    coDriverLeft.button(1).onTrue(new SetOrientation(0));
    coDriverLeft.button(2).onTrue(new SetOrientation(1));

    driverButtons.button(7)
        .onTrue(new SetWristState(WristStates.FOURTYFIVE, ClosedLoopSlot.kSlot0).andThen(new WaitCommand(1))
            .andThen(new SetElevatorState(ElevatorStates.LEVEL3)));

    // driverButtons.button(-1).whileTrue(new ScoreCoral()); // TODO - Implement
    // NOTE - This is just for testing:
    driverButtons.button(5).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
    driverButtons.button(6).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
    driverButtons.button(7).onTrue(new SetElevatorState(ElevatorStates.LEVEL1));
    driverButtons.button(8).onTrue(new SetElevatorState(ElevatorStates.LEVEL2));
    driverButtons.button(9).onTrue(new SetElevatorState(ElevatorStates.LEVEL3));
    driverButtons.button(10).onTrue(new SetElevatorState(ElevatorStates.LEVEL4));
  }
}
