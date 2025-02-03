package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class fakeAlignSource extends Command {
    public fakeAlignSource() {
        SmartDashboard.putBoolean("align to source", true);
    }
}