package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class CoralPlaceAndAlgaeReefClear extends SequentialCommandGroup{
    public CoralPlaceAndAlgaeReefClear(){
        addCommands(
            new ScoreCoral(),
            new InstantCommand(() -> {
                    Swerve.getInstance().setDriveState(DriveState.Reverse);
                }),
            new ClearAlgae()
        );
    }
}
