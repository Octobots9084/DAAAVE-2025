package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class ButtonConfig {
  CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

  public void initTeleop() {
    driverButtons.button(7).onTrue(new SetElevatorState(ElevatorStates.LEVEL4));
    driverButtons.button(9).onTrue(new SetElevatorState(ElevatorStates.LEVEL3));
    driverButtons.button(11).onTrue(new SetElevatorState(ElevatorStates.LEVEL2));
  }
}
