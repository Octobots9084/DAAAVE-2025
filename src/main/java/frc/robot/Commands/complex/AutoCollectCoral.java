package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCollectCoral extends SequentialCommandGroup {
    public AutoCollectCoral() {
        addCommands(
                new CollectCoral());
    }
}
