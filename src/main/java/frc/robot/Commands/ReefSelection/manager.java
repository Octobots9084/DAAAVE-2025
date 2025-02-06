package frc.robot.Commands.ReefSelection;

public class manager {
  public static int[] LastButtonPos = new int[2];
  private static boolean[] level = {false, false, false, false};
  private static boolean[][] reef = new boolean[6][2];
  public static GetReefSide joystick = new GetReefSide();

  public enum joystickState {
    AB,
    CD,
    EF,
    GH,
    IH,
    KL,
    NONE
  };

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

  public static boolean getReefPos(int row, int col) {
    return reef[row][col];
  }

  public static boolean[][] getReef() {
    return reef;
  }

  public static void setLevel(int targetLevel, boolean state) {
    level[targetLevel] = state;
  }

  public static boolean[] getLevels() {
    return level;
  }

  public static boolean getLevelPos(int pos) {
    return level[pos];
  }

  public static void clearLevels() {
    for (int i = 0; i < level.length; i++) {
      level[i] = false;
    }
  }
}
