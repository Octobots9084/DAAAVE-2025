package frc.robot.Commands.complex;

public class BallThrow implements SequentialCommandGroup  /**//**/ {
    
    public BallThrow() {
        addCommands(
        new groundAlgae(),
        new ParallelCommandGroup(
            new SetWristState(WristStates.PREP),
            new SetElevatorState(ElevatorStates.L4)
        ),
        new SetCoralRollers(CoralRollersState.ALGAEOUTPUT),
        new RobotSafeState();
        ); 
    }
    
}