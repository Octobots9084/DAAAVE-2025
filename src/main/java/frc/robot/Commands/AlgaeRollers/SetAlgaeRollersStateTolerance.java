package frc.robot.Commands.AlgaeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollers;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;

public class SetAlgaeRollersStateTolerance extends Command {
    private AlgaeRollersStates targetState;
    private double tolerance;
    private AlgaeRollers algaeRollers = AlgaeRollers.getInstance();
    public SetAlgaeRollersStateTolerance(AlgaeRollersStates targetState,double tolerance) {
        this.targetState = targetState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize(){
        algaeRollers.setState(targetState);
    }

    @Override
    public boolean isFinished(){
        return algaeRollers.isAtState(targetState,tolerance);
    }
}
