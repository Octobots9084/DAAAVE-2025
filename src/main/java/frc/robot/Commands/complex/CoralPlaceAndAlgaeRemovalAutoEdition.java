package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Vision.AlignVision;

public class CoralPlaceAndAlgaeRemovalAutoEdition extends Command{
    ElevatorStates targetLevel;
    ReefTargetSide targetSide;
    ReefTargetOrientation targetOrientation;

    public CoralPlaceAndAlgaeRemovalAutoEdition(ElevatorStates targetLevel, ReefTargetSide targetSide, ReefTargetOrientation targetOrientation) {
        this.targetLevel = targetLevel;
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
    }

    @Override
    public void initialize() {
        AlignVision.setPoleLevel(targetLevel);
        AlignVision.setPoleSide(targetSide);
        AlignVision.setReefOrientation(targetOrientation);
    }

    @Override
    public void execute() {
        new CoralPlaceAndAlgaeReefClear();
    }
}
