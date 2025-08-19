package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Commands.complex.BargeThrow;

public class AlignBarge extends Command {
    private Swerve swerve = Swerve.getInstance();
    // TODO implement this using vlaues from elevator and swerve

    @Override
    public void initialize() {
        swerve.setDriveState(DriveState.AlignBarge);
    }

    @Override
    public boolean isFinished() {
        // check if the robot is aligned
        return AlignVision.getInstance().isAligned();
    }
}