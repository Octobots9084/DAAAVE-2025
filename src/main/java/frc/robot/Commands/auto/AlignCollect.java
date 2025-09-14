package frc.robot.Commands.auto;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.States.AlignState;
import frc.robot.Constants;
import frc.robot.Commands.complex.AlignSource;
import frc.robot.Commands.complex.CollectCoral;
import frc.robot.Commands.complex.Intake;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignSourceAuto;
import frc.robot.Subsystems.Vision.AlignVision;

public class AlignCollect extends Command {
    private Debouncer debouncer;
    private boolean shouldTakeControlFromDrivers = false;
    private double frames;
    public AlignCollect() {
        debouncer = new Debouncer(0.05);
    }

    @Override
    public void initialize() {
        Swerve.getInstance().setDriveState(DriveState.AlignSource);
        CommandScheduler.getInstance().schedule(new Intake());
        frames = 0;
    }

    @Override
    public void execute() {
        // shouldTakeControlFromDrivers =
        // debouncer.calculate(AlignSourceAuto.getInstance().wrenchControlFromDriversForSourceAlign());
        // if (shouldTakeControlFromDrivers) {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignVision.getInstance().getAlignSourceSide()));
        // }
        frames += 1;
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Constants.simMode) {
            return true;
        }
        // return AlignVision.getInstance().isAligned();
        return debouncer.calculate(CoralRollers.getInstance().clawFrontSensorTriggered());
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().setDriveState(DriveState.Manual);
        frames *= 0.02;
        Constants.intakeTimes.add(frames);
        double total = 0;
        for(double i: Constants.intakeTimes){
            total += i;
        }
        SmartDashboard.putNumber("Average Intake Time Auto", total/Constants.intakeTimes.size());
    }
}
