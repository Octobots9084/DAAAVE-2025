package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.States.*;
import frc.robot.Subsystems.Swerve.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetOrientation extends InstantCommand {
    private int side;
    private Swerve swerve = Swerve.getInstance();

    public SetOrientation(int side) {
        System.out.println("\n\n\n\n\n\n\n\n");
        this.side = side;
    }

    @Override
    public void initialize() {
        ReefTargetOrientation reefState = manager.joystick.joystickPos();
        int reefStatePos = reefState.ordinal();
        SmartDashboard.putNumber("this is a number and if this is correct I will be happy", reefStatePos);
        if (reefState != ReefTargetOrientation.NONE) {
            manager.LastButtonPos[0] = reefStatePos;
            // replaces the terrible switching at last moment code
            if (manager.LastButtonPos[0] > 1 && manager.LastButtonPos[0] < 5) {
                manager.LastButtonPos[1] = (side == 0 ? 1 : 0);
            } else {
                manager.LastButtonPos[1] = side;
            }
            manager.clearReef();
            manager.setReef(reefStatePos, manager.LastButtonPos[1], true);

            ReefTargetSide reefSide = ReefTargetSide.values()[manager.LastButtonPos[1]];
            ReefTargetOrientation reefOrientation = ReefTargetOrientation.values()[manager.LastButtonPos[0]];

            SmartDashboard.putString("Selected Reef Side", reefSide.name());
            SmartDashboard.putString("Selected Reef Orientation", reefOrientation.name());

            swerve.setReefTargetSide(reefSide);
            swerve.setReefTargetOrientation(reefOrientation);
            SmartDashboard.putString("stuff", manager.LastButtonPos[0] + " " + manager.LastButtonPos[1]);
        }
    }
}
