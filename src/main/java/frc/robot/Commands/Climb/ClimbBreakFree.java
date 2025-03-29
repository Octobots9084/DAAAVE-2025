package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbStates;

public class ClimbBreakFree extends InstantCommand {
    Climb climbInstance;
    double startTime;
    
    public ClimbBreakFree() {
        climbInstance = Climb.getInstance();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return climbInstance.isAtState(ClimbStates.BreakFree);
    }

    @Override
    public void end(boolean interrupted) {}

}