package frc.robot.Commands.auto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.AlignVision;

public class testAlignInAuto extends Command {
    ReefTargetOrientation targetOrientation;
    ReefTargetSide targetSide;
    boolean alignAlgae;
    private Debouncer debouncer;

    public testAlignInAuto(ReefTargetSide targetSide, ReefTargetOrientation targetOrientation, boolean alignAlgae) {
        this.targetSide = targetSide;
        this.targetOrientation = targetOrientation;
        this.alignAlgae = alignAlgae;
    }

    @Override
    public void initialize() {
        debouncer = new Debouncer(0.05);

        AlignVision.setReefOrientation(targetOrientation);

        if (!alignAlgae) {
            AlignVision.setPoleSide(targetSide);
        }//not sure if this is how i make it go to center, its 2 am
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(AlignVision.getInstance().getAlignChassisSpeeds(AlignState.Reef));
    }

    @Override
    public boolean isFinished() {
        if (Constants.currentMode == Mode.SIM) {
            return true;
        }

        return debouncer.calculate(AlignVision.getInstance().isAligned());
    }


    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds());
    }
}