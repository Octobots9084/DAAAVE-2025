package frc.robot.Commands.ReefSelection;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States.*;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AutoSelector;

public class AutoSelectorSidePickerWhileTrue extends Command {
    public void execute()
    {
        AutoSelector.pickASide();
    }
}
