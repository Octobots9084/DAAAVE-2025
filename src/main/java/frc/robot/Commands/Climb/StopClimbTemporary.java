package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Climb.Climb;

public class StopClimbTemporary extends InstantCommand{
    public void initialize(){
        Climb.getInstance().setTalonVoltage(0);
    }
}
