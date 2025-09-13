package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveIO;

public class setClimbChasisSpeed extends InstantCommand{
    private SwerveIO swerve = Swerve.getInstance().getIo();
    @Override
    public void initialize() {
        if(swerve.getMaxSpeed() == 12){
            swerve.setMaxSpeed(1);
            SmartDashboard.putBoolean("IsFast",false);
        }else{
            swerve.setMaxSpeed(12);
            SmartDashboard.putBoolean("IsFast",true);
        }
    }
}
