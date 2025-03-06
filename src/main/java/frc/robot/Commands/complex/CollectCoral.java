package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.complex.collectCoral.WaitForClawFrontSensor;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class CollectCoral extends SequentialCommandGroup {
  public CollectCoral() {
    addCommands(
        new SetCoralRollersState(CoralRollersState.INTAKING),
        new WaitForClawFrontSensor().withTimeout(3),
        new WaitCommand(0.1),
        new SetCoralRollersState(CoralRollersState.STOPPED));
  }

}
