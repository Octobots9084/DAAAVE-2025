package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbStates;

public class SetClimbState extends InstantCommand {
    ClimbStates targetState;

    public SetClimbState(ClimbStates state) {
        targetState = state;
    }

    @Override
    public void initialize() {
        Climb.getInstance().setState(targetState);
    }

}
