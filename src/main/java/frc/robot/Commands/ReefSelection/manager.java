package frc.robot.Commands.ReefSelection;

import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Elevator.ElevatorStates;

public class manager {
    public static int[] LastButtonPos = new int[2];
    private static boolean[][] reef = new boolean[6][2];
    public static GetReefSide joystick = new GetReefSide();
    public static ElevatorStates level = ElevatorStates.LEVEL1;
    public static ReefTargetOrientation orientation = ReefTargetOrientation.NONE;
    public static ReefTargetSide selectedReefSide = ReefTargetSide.ALGAE;
    public static boolean isAuto;

    public static void clearReef() {
        for (int i = 0; i < reef.length; i++) {
            for (int j = 0; j < reef[0].length; j++) {
                reef[i][j] = false;
            }
        }
    }

    public static void setReef(int row, int col, boolean state) {
        reef[row][col] = state;
    }

    public static void setReefSide(int row, boolean state){
        reef[row][0] = state;
        reef[row][1] = state;
    }

    public static boolean getReefPos(int row, int col) {
        return reef[row][col];
    }

    public static boolean[][] getReef() {
        return reef;
    }
}
