package frc.robot.Commands.Climb;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbStates;

public class SetClimbState extends InstantCommand {
    ClimbStates targetState;
    ClosedLoopSlot slot;

    public SetClimbState(ClimbStates state, ClosedLoopSlot slot) {
        targetState = state;
        this.slot = slot;
    }

    @Override
    public void initialize() {
        Climb.getInstance().setState(targetState, slot);
    }

}
