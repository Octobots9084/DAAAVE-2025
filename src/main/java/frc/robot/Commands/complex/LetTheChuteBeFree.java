package frc.robot.Commands.complex;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LetTheChuteBeFree extends InstantCommand{
    private Servo servo;
    public LetTheChuteBeFree(){
        //TODO set actual PWM channel on robot rio
        servo = new Servo(0);
    }

    @Override
    public void initialize(){
        servo.setAngle(30);
    }
}
