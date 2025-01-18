package frc.robot.Commands.complex;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;

//this is where we set alignment to true

//"TODO replace this with actual swerve checking"

public class ProcessorAlignment extends Command{
    
    private boolean aligned;
    public ProcessorAlignment () {
        aligned = true;
    }

    @Override
    public boolean isFinished() {
        return aligned;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        CommandScheduler.getInstance().schedule(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
        super.end(interrupted);
    }
}

/*
constructor parameters -> algae roller state

intialize -> 
aligned
check if aligned is true
if it is aligned run the set algaet roller state command
 */