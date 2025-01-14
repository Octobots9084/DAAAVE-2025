package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class SetWristStateTolerance extends Command {
    private WristStates targetState;
    private double tolerance;
    private Wrist wrist = Wrist.getInstance();
    public SetWristStateTolerance(WristStates targetState,double tolerance) {
        this.targetState = targetState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize(){
        wrist.setState(targetState);
    }

    @Override
    public boolean isFinished(){
        return wrist.isAtState(targetState,tolerance);
    }
}
