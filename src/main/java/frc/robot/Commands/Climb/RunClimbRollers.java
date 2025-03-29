package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.Climb.ClimbStates;

public class RunClimbRollers extends InstantCommand {
    ClimbStates targetState;
    double stalledCylces = 0;
    Climb climbInstance;

    public RunClimbRollers() {
        climbInstance = Climb.getInstance();
    }

    @Override
    public void initialize() {
        climbInstance.setTalonVoltage(10);
    }

    @Override
    public void execute() {
        if (climbInstance.talonIsStalled()) {
            stalledCylces++;
        } else {
            stalledCylces = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return stalledCylces > 250;
    }

    @Override
    public void end(boolean interrupted) {
        climbInstance.setTalonVoltage(0);
    }

}
