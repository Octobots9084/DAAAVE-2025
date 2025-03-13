package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.States;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;

public class CoralPlaceAndAlgaeReefClear extends SequentialCommandGroup{
    public CoralPlaceAndAlgaeReefClear(){
        States.ReefTargetOrientation pos = Swerve.getInstance().getReefTargetOrientation();
        States.ReefTargetOrientation reefPos = ReefTargetOrientation.NONE;
        Boolean up = (pos == reefPos.AB || pos == reefPos.EF || pos == reefPos.IJ);
        addCommands(
            new ScoreCoral(),
            new InstantCommand(() -> {
                Swerve.getInstance().setDriveState(DriveState.AlignReef);
            }),
            new WaitUntilCommand(() -> AlignVision.getInstance().isAligned()),
            new ConditionalCommand(new RemoveAlgaeTop(), new RemoveAlgaeBottom(),() -> up)
        );
    }
}
