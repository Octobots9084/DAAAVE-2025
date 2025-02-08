package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.complex.collectcoral.WaitForClawFrontSensor;
import frc.robot.Commands.complex.collectcoral.WaitForCoralDetected;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class CollectCoral extends SequentialCommandGroup {
  public CollectCoral() {
    addCommands(
        new SetCoralRollersState(CoralRollersState.INTAKING),
        new WaitForClawFrontSensor(),
        new SetCoralRollersState(CoralRollersState.STOPPED));
  }

}
