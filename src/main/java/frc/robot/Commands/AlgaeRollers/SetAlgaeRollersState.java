package frc.robot.Commands.AlgaeRollers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollers;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;

public class SetAlgaeRollersState extends InstantCommand {
    private AlgaeRollersStates targetState;
    public SetAlgaeRollersState(AlgaeRollersStates targetState) {
        this.targetState = targetState;
    }

    @Override
    public void initialize(){
        AlgaeRollers.getInstance().setState(targetState);
    }
}
