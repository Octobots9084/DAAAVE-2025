package frc.robot.Commands.swerve.drivebase;



import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

public class SetDriveState extends InstantCommand {
    DriveState driveState;
    public SetDriveState(DriveState driveState){
        this.driveState = driveState;
    }
    public void initialize(){
        Swerve.getInstance().setDriveState(driveState);
    }
}
