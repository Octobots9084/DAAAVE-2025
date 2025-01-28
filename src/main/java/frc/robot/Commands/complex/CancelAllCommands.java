package frc.robot.Commands.Complex;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CancelAllCommands extends InstantCommand {
    @Override 
    public void initialize(){
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();

    }
    
}