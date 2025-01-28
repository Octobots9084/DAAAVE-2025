package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.AlgaeRollers.SetAlgaeRollersState;
import frc.robot.Commands.Complex.CancelAllCommands;
import frc.robot.Commands.CoralRollers.SetCoralRollersState;
import frc.robot.Commands.Elevator.SetElevatorState;
import frc.robot.Commands.Complex.ScoreCoral;
import frc.robot.Commands.Complex.SetTargetReefLevel;
import frc.robot.Subsystems.AlgaeRollers.AlgaeRollersStates;
import frc.robot.Subsystems.CoralRollers.CoralRollersState;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class ButtonConfig {
  static CommandJoystick driverLeft = ControlMap.DRIVER_LEFT;
  static CommandJoystick driverRight = ControlMap.DRIVER_RIGHT;
  CommandJoystick driverButtons = ControlMap.DRIVER_BUTTONS;
  CommandJoystick coDriverLeft = ControlMap.CO_DRIVER_LEFT;
  CommandJoystick coDriverRight = ControlMap.CO_DRIVER_RIGHT;
  CommandJoystick coDriverButtons = ControlMap.CO_DRIVER_BUTTONS;

  public void initTeleop() {

    driverButtons
        .button(11)
        .onTrue(
            new InstantCommand(
                () -> {
                  Swerve.getInstance().zeroGyro();
                }));
    // reef align

    // source align
    //driverButtons.button(1).whileTrue(new AlignSource());

    // climb(no commands yet)
    // driverButtons
    //     .button(16)
    //     .onTrue(); (change for switch)

    // processor align? (4)
    //driverButtons.button(4).whileTrue(new AlignSource());

    //driverButtons.button(-1).whileTrue(new AlignReef().andThen(new SetCoralRollersState(CoralRollersState.OUTPUT)));

    // driverButtons.button(-1).whileTrue(new ScoreCoral()); // TODO - Implement
    // NOTE - This is just for testing:
    driverButtons.button(5).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
    driverButtons.button(6).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
    driverButtons.button(7).onTrue(new SetTargetReefLevel(ElevatorStates.LEVEL1));
    driverButtons.button(8).onTrue(new SetTargetReefLevel(ElevatorStates.LEVEL2));
    driverButtons.button(9).onTrue(new SetTargetReefLevel(ElevatorStates.LEVEL3));
    driverButtons.button(10).onTrue(new SetTargetReefLevel(ElevatorStates.LEVEL4));


    //when the Climb system switches on and button 2 we want to climb
    //driverButtons.button(16).and(driverButtons.button(2)).onTrue();
    //coDriverButtons.button(16).and(coDriverButtons.button(2)).onTrue();
    
    // when the climb system switches on and button 3 we want to RESET
    //driverButtons.button(16).and(driverButtons.button(3)).onTrue();
    //coDriverButtons.button(16).and(driverButtons.button(3)).onTrue();

    driverRight.button(1).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
    driverRight.button(2).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
    coDriverRight.button(1).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.INTAKE));
    coDriverRight.button(2).onTrue(new SetAlgaeRollersState(AlgaeRollersStates.OUTPUT));
    
    
        //TO DO: drive button 7 needs to left alingn using the swerver sub system

    // driver button 1 is align to coral source 
    //driverButtons.button(16).negate().and(driverButtons.button(1)).onTrue();
    // coDriverButtons.button(16).negate().and(driverButtons.button(1)).onTrue();
   
   
    // driver button 2 is align to processer 
    //coDriverButtons.button(16).negate().and(coDriverButtons.button(2)).onTrue();
    //driverButtons.button(16).negate().and(driverButtons.button(2)).onTrue();


    //button 3 is avaible for any command
    //driverButtons.button(16).negate().and(driverButtons.button(3)).onTrue();
    //coDriverButtons.button(16).negate().and(coDriverButtons.button(3)).onTrue();
    
    

    // driver button 4 is PANIC (releases all game pieces)
    // driverButtons.button(4).onTrue();
    // coDriverButtons.button(4).onTrue();

// driver button 5 is cancel all commands
    driverButtons.button(6).onTrue(new CancelAllCommands());
    coDriverButtons.button(6).onTrue(new CancelAllCommands());
    // driver button 6 is ZERO gyro
    driverButtons.button(11).onTrue(new InstantCommand(() -> {Swerve.getInstance().zeroGyro();}));
  }
}
