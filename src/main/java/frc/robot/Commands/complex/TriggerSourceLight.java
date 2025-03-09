package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Lights.Light;
import frc.robot.Subsystems.Vision.AlignVision;

public class TriggerSourceLight extends Command {
    @Override
    public void initialize() {
        AlignVision.isCollecting = true;
    }

    @Override
    public void end(boolean interrupted) {
        AlignVision.isCollecting = false;
    }
}
