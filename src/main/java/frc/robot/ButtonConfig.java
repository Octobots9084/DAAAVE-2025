package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.ReefSelection.ReefLevelSelection;
import frc.robot.Commands.ReefSelection.SetOrientation;
import frc.robot.Commands.complex.ScoreCoral;
import frc.robot.Subsystems.Elevator.Elevator;
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

    // coDriverLeft.button(1).onTrue(new SetOrientation(0));
    // coDriverLeft.button(2).onTrue(new SetOrientation(1));

    // coDriverButtons.button(7).onTrue(new ReefLevelSelection(2));
    // coDriverButtons.button(9).onTrue(new ReefLevelSelection(1));
    // coDriverButtons.button(11).onTrue(new ReefLevelSelection(0));

    // driverRight.button(1).whileTrue(new ScoreCoral(Elevator.getInstance().getReefTargetLevel(),
    //     Swerve.getInstance().getReefTargetSide(), Swerve.getInstance().getReefTargetOrientation()));
  }
}
