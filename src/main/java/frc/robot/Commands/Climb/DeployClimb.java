package frc.robot.Commands.Climb;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbStates;

public class DeployClimb extends InstantCommand {
    Climb climbInstance;
    double startTime;

    public DeployClimb() {
        climbInstance = Climb.getInstance();
    }

    @Override
    public void initialize() {
        climbInstance.setState(ClimbStates.Deployed, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return climbInstance.isAtState(ClimbStates.Deployed);
    }

    @Override
    public void end(boolean interrupted) {}

}
