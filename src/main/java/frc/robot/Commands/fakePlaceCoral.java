package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class fakePlaceCoral extends Command {
    public fakePlaceCoral() {
        SmartDashboard.putBoolean("place coral L4", true);
    }
}