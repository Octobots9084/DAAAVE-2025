package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ButtonLogger extends InstantCommand{

    public ButtonLogger (String log, boolean driver, int buttonNumber) {
        if (driver) {
            Logger.recordOutput(("Driver pressed button " + buttonNumber), log);
        } else {
            Logger.recordOutput(("Co-Driver pressed button " + buttonNumber), log);
        }
    }
}
