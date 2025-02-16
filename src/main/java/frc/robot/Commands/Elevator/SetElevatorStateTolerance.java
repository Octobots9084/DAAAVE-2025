package frc.robot.Commands.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class SetElevatorStateTolerance extends Command {
    private ElevatorStates targetState;
    private double tolerance;
    private Elevator elevator = Elevator.getInstance();

    private static double kDt = 0.02;

    private final TrapezoidProfile elevatorProfile;

    private TrapezoidProfile.State elevatorGoal;
    private TrapezoidProfile.State elevatorCurrentPoint;

    public SetElevatorStateTolerance(ElevatorStates targetState, double tolerance) {
        this.targetState = targetState;
        this.tolerance = tolerance;
        this.elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(72, 50));
        this.elevatorCurrentPoint = new TrapezoidProfile.State(elevator.getPosition(), 0);
        SmartDashboard.putNumber("Elevator Current Position", elevator.getPosition());
        this.elevatorGoal = new TrapezoidProfile.State(targetState.position, 0);
    }

    @Override
    public void initialize() {
        elevator.setState(targetState);
    }

    @Override
    public boolean isFinished() {
        return elevator.isAtState(targetState, tolerance);
    }

    @Override
    public void execute() {
        elevatorCurrentPoint = elevatorProfile.calculate(kDt, elevatorCurrentPoint, elevatorGoal);
        SmartDashboard.putNumber("Elevator Set Point", elevatorCurrentPoint.position);
        SmartDashboard.putNumber("Elevator Velocity", elevatorCurrentPoint.velocity);
        elevator.getElevatorIo().setPosition(elevatorCurrentPoint.position);
    }
}
