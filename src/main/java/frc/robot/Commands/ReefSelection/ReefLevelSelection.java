package frc.robot.Commands.ReefSelection;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ReefLevelSelection extends InstantCommand {
  private int level;

  public ReefLevelSelection(int level) {
    this.level = level - 1;
  }

  @Override
  public void initialize() {
    manager.clearLevels();
    manager.setLevel(level, true);
  }
}
