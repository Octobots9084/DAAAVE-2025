package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Climb.ClimbStates;

public class ClimbSequence extends SequentialCommandGroup {
    public ClimbSequence() {
        addCommands(
            new ClimbBreakFree(),
            new ParallelCommandGroup(new DeployClimb(), new RunClimbRollers()),
            new SetClimbState(ClimbStates.Climbing)
        );
    }
}
