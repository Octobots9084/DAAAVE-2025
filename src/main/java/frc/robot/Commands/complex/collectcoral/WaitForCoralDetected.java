package frc.robot.Commands.complex.collectcoral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class WaitForCoralDetected extends Command {
    CoralRollers coralRollers;
    Elevator elevator;

    public WaitForCoralDetected() {
        coralRollers = CoralRollers.getInstance();
        elevator = Elevator.getInstance();
    }

    public boolean isFinished() {
        return (!coralRollers.clawFrontSensorTriggered() && elevator.getReefTargetLevel() == ElevatorStates.LOW)
                && (coralRollers.clawBackSensorTriggered()
                        || coralRollers.chuteSensorTriggered());
    }

}
