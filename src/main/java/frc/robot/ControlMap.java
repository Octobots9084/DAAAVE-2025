package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class ControlMap {
  public static final CommandJoystick DRIVER_LEFT = new CommandJoystick(0);
  public static final CommandJoystick DRIVER_RIGHT = new CommandJoystick(1);
  public static final CommandJoystick DRIVER_BUTTONS = new CommandJoystick(2);
  public static final CommandJoystick CO_DRIVER_LEFT = new CommandJoystick(3);
  public static final CommandJoystick CO_DRIVER_RIGHT = new CommandJoystick(4);
  public static final CommandJoystick CO_DRIVER_BUTTONS = new CommandJoystick(5);

  private ControlMap() {}
}
