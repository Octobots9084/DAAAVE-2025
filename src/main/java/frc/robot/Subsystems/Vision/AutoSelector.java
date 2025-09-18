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

        if (!Constants.isBlueAlliance) {
            if (swerve.getPose().getY() - 4.01 > 0)
                angle = 90 + Math.toDegrees(Math.atan((swerve.getPose().getX() - 13.06)/(swerve.getPose().getY() - 4.01)));
            else
                angle = 270 + Math.toDegrees(Math.atan((swerve.getPose().getX() - 13.06)/(swerve.getPose().getY() - 4.01)));

        }
        else {
            if (swerve.getPose().getY() - 4.01 > 0)
                angle = 270 - Math.toDegrees(Math.atan((swerve.getPose().getX() - 4.50)/(swerve.getPose().getY() - 4.01)));
            else
                angle = 90 - Math.toDegrees(Math.atan((swerve.getPose().getX() - 4.50)/(swerve.getPose().getY() - 4.01)));
        }
        SmartDashboard.putNumber("align Angle", angle);
        angle = Math.round(angle/60)*60;
        if(angle == 0){
            manager.clearReef();
            manager.setReef(0,manager.LastButtonPos[1],true);
            Swerve.getInstance().setReefTargetOrientation(ReefTargetOrientation.AB);
        }if(angle == 60){
            manager.clearReef();
            manager.setReef(1,manager.LastButtonPos[1],true);
            Swerve.getInstance().setReefTargetOrientation(ReefTargetOrientation.CD);
        }if(angle == 120){
            manager.clearReef();
            manager.setReef(2,manager.LastButtonPos[1],true);
            Swerve.getInstance().setReefTargetOrientation(ReefTargetOrientation.EF);
        }if(angle == 180){
            manager.clearReef();
            manager.setReef(3,manager.LastButtonPos[1],true);
            Swerve.getInstance().setReefTargetOrientation(ReefTargetOrientation.GH);
        }if(angle == 240){
            manager.clearReef();
            manager.setReef(4,manager.LastButtonPos[1],true);
            Swerve.getInstance().setReefTargetOrientation(ReefTargetOrientation.IJ);
        }if(angle == 300){
            manager.clearReef();
            manager.setReef(5,manager.LastButtonPos[1],true);
            Swerve.getInstance().setReefTargetOrientation(ReefTargetOrientation.KL);
        }
        // CommandScheduler.getInstance().schedule(new RotateToAngle((int)angle));
    }
}
