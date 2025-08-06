package frc.robot.Commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.complex.groundAlgae;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Vision.PieceVisionCamera;

public class GroundAlgaeAlign extends Command{
    private PieceVisionCamera algaeCamera =  new PieceVisionCamera("Velociraptor", new Transform3d()); //TODO rename the the new camera to be Velociraptor
    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
    @Override
    public void initialize() {
        commandScheduler.schedule(new groundAlgae());
    }

    @Override
    public void execute() {
        Swerve.getInstance().driveRobotRelative(new ChassisSpeeds(1,0, algaeCamera.getCenterOffset())); //TODO tune the exact turn and forward rates
    }

    @Override 
    public boolean isFinished() {
        if (!algaeCamera.hasTargets() || CoralRollers.getInstance().isStalled()){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        commandScheduler.schedule(new SetCoralRollersState(CoralRollersState.ALGAEHOLD));
    }
    
}
