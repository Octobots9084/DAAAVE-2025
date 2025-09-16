package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Commands.ReefSelection.SetTargetReefSide;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Commands.complex.RotateToAngle;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.Subsystems.Swerve.Swerve;

public class AutoSelector {
    public static void pickASide(){
        Swerve swerve = Swerve.getInstance();
        double angle = 0;

        if (Constants.isBlueAlliance) {
            angle = 180 + Math.toDegrees(Math.tan((swerve.getPose().getX() - 13.06)/(swerve.getPose().getY() - 4.01)));
        }
        else {
            angle = 180 + Math.toDegrees(Math.tan((swerve.getPose().getX() - 4.50)/(swerve.getPose().getY() - 4.01)));
        }
        angle = Math.round(angle/60)*60;
        if(angle == 0){
            manager.clearReef();
            manager.setReefSide(0,true);
        }if(angle == 60){
            manager.clearReef();
            manager.setReefSide(1,true);
        }if(angle == 120){
            manager.clearReef();
            manager.setReefSide(2,true);
        }if(angle == 180){
            manager.clearReef();
            manager.setReefSide(3,true);
        }if(angle == 240){
            manager.clearReef();
            manager.setReefSide(4,true);
        }if(angle == 300){
            manager.clearReef();
            manager.setReefSide(5,true);
        }
        CommandScheduler.getInstance().schedule(new RotateToAngle((int)angle));
    }
}
