package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class SetElevatorStateTolerance extends Command {
    private ElevatorStates targetState;
    private double tolerance;
    private Elevator elevator = Elevator.getInstance();
    public SetElevatorStateTolerance(ElevatorStates targetState,double tolerance) {
        this.targetState = targetState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize(){
        elevator.setState(targetState);
    }

    @Override
    public boolean isFinished(){
        return elevator.isAtState(targetState,tolerance);
    }
}
