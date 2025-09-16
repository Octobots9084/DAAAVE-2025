package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Elevator.*;

public class ShowSelection {
    private static Elevator elevator = Elevator.getInstance();

    @SuppressWarnings("static-access")
    public static void displayReefSelection() {
        if(!manager.isAuto){
            int reefState = manager.joystick.joystickPos().ordinal();
            if (reefState == 6) {
                manager.clearReef();
                manager.setReef(manager.LastButtonPos[0], manager.LastButtonPos[1], true);
            } else {
                manager.clearReef();
                manager.setReef(reefState, 0, true);
                manager.setReef(reefState, 1, true);
                manager.setReef(manager.LastButtonPos[0], manager.LastButtonPos[1], true);
            }
        }

        for (int i = 0; i < manager.getReef().length; i++) {
            for (int j = 0; j < manager.getReef()[0].length; j++) {
                SmartDashboard.putBoolean(i * 2 + j + "", manager.getReefPos(i, j));
            }
        }

        for (int i = 1; i < 5; i++) {
            SmartDashboard.putBoolean("L" + i, manager.level == ElevatorStates.values()[i]);
        }
    }
}
