package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ButtonLogger extends InstantCommand{

    public ButtonLogger (String log, boolean driver, int buttonNumber) {
        Logger.recordOutput(("Button " + buttonNumber + ", Driver? " + driver), log);
    }
}
