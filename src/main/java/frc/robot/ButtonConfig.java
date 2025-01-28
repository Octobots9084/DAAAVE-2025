package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.complex.AlignReef;
import frc.robot.Commands.complex.AlignSource;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

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
    driverButtons.button(2).whileTrue(new AlignReef());
    driverButtons
        .button(4)
        .onTrue(
            new InstantCommand(
                () -> {
                  AlignVision.setReefOrientation(ReefTargetOrientation.IJ);
                  AlignVision.setPoleSide(ReefTargetSide.LEFT);
                  AlignVision.setPoleLevel(ReefTargetLevel.L1);
                }));
    driverButtons
        .button(3)
        .onTrue(
            new InstantCommand(
                () -> {
                  AlignVision.setReefOrientation(ReefTargetOrientation.IJ);
                  AlignVision.setPoleSide(ReefTargetSide.RIGHT);
                  AlignVision.setPoleLevel(ReefTargetLevel.L1);
                }));

    // source align
    driverButtons.button(1).whileTrue(new AlignSource());

    // climb(no commands yet)
    // driverButtons
    //     .button(16)
    //     .onTrue(); (change for switch)

    // processor align? (4)
    // driverButtons.button(4).whileTrue(new AlignSource());

    driverButtons
        .button(-1)
        .whileTrue(new AlignReef().andThen(new SetCoralRollersState(CoralRollersState.REJECTING)));
  }
}
