package frc.robot.Commands.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;

public class SetElevatorStateTolerance extends Command {
    private ElevatorStates targetState;
    private double tolerance;
    private Elevator elevator = Elevator.getInstance();

    public SetElevatorStateTolerance(ElevatorStates targetState, double tolerance) {
        this.targetState = targetState;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        boolean frontClear = (AlignVision.getInstance().getLeftLidarDistance() > 0.4 || AlignVision.getInstance().getRightLidarDistance() > 0.4);
        if ((frontClear || Wrist.getInstance().isAtState(WristStates.PREP, 0.01)) && elevator.getTargetState() != targetState) 
            elevator.setState(targetState);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtState(targetState, tolerance);
    }

}
