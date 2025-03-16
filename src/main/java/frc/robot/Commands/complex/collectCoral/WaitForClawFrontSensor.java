package frc.robot.Commands.complex.collectCoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;

public class WaitForClawFrontSensor extends Command {
    CoralRollers coralRollers;
    boolean isStalled = false;

    public WaitForClawFrontSensor() {
        coralRollers = CoralRollers.getInstance();
    }

    @Override
    public void execute() {
        if (coralRollers.isStalled() && !isStalled) {
            isStalled = true;
            coralRollers.setState(CoralRollersState.OUTPUT);
        } else if (!coralRollers.isStalled() && isStalled) {
            isStalled = false;
            coralRollers.setState(CoralRollersState.INTAKING);
        }
    }

    public boolean isFinished() {
        return coralRollers.clawFrontSensorTriggered();
    }

}