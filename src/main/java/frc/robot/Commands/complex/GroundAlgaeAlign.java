package frc.robot.Commands.complex;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.PieceVisionCamera;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIOSystem;
import frc.robot.Subsystems.Vision.VisionSubsystem;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;

public class GroundAlgaeAlign extends Command{
    private PieceVisionCamera algaeCamera  = VisionSubsystem.getInstance().io.getAlgaeCamera();
    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
    @Override
    public void initialize() {
        commandScheduler.schedule(new groundAlgae());
        commandScheduler.schedule(new WaitCommand(0.5));
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds((algaeCamera.getOffsetPitch()*0.05)+0.5,0, algaeCamera.getCenterOffset()/-10));
    }

    @Override 
    public boolean isFinished() {
        if (!algaeCamera.hasTargets() && CoralRollers.getInstance().isStalled()){
            return true;
        }
        return false;
    }
}
