package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Vision.AlignVision;

public class WaitForAlign extends Command{
    @Override
    public boolean isFinished() {
        if (AlignVision.getInstance().isAligned())
            return true;
        return false;
    }
}
