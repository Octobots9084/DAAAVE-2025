package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;

public class AlignSource extends Command {

    Swerve swerve = Swerve.getInstance();
    private double frames;

    @Override
    public void initialize() {
        swerve.setDriveState(DriveState.AlignSource);
        CommandScheduler.getInstance().schedule(new Intake());
        frames = 0;
    }

    @Override
    public void execute() {
        if(AlignVision.getInstance().isAligned()){
            frames += 1;
        }
    }

    @Override
    public boolean isFinished() {
        // if the coral is inside the robot return true
        return CoralRollers.getInstance().HasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDriveState(DriveState.Manual);
        frames *= 0.02;
        Constants.intakeTimes.add(frames);
        // double total = 0;
        // for(double i: Constants.intakeTimes){
        //     total += i;
        // }
        SmartDashboard.putNumber("Average Intake Time Teleop", frames);//total/Constants.intakeTimes.size());
    }
}
