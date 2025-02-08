package frc.robot.Commands.Wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Wrist.*;


public class OffsetWristEncoder extends Command{
    double encoderPosition;
    @Override
    public void execute(){
        encoderPosition = Wrist.getInstance().getWristMotor().getEncoder().getPosition();
        Wrist.getInstance().setOffset(encoderPosition-WristStates.VERTICAL.wristPosition);
    }
}
