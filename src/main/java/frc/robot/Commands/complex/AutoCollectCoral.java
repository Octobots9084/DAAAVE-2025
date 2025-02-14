package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.complex.collectCoral.WaitForCoralDetected;

public class AutoCollectCoral extends SequentialCommandGroup {
    public AutoCollectCoral() {
        addCommands(
                new WaitForCoralDetected(),
                new CollectCoral());
    }
}
