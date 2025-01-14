package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class SetWristState extends InstantCommand {
    private WristStates targetState;
    public SetWristState(WristStates targetState) {
        this.targetState = targetState;
    }

    @Override
    public void initialize(){
        Wrist.getInstance().setState(targetState);
    }
}
