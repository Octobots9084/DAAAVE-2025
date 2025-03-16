// package frc.robot.Commands.complex.collectCoral;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import frc.robot.Subsystems.CoralRollers.CoralRollers;
// import frc.robot.Subsystems.CoralRollers.CoralRollersState;
// import frc.robot.Subsystems.Vision.AlignVision;

// public class WaitForClawFrontSensor extends Command {
// CoralRollers coralRollers;
// boolean isStalled = false;

// public WaitForClawFrontSensor() {
// coralRollers = CoralRollers.getInstance();
// }

// @Override
// public void execute() {
// if (coralRollers.isStalled() && !isStalled) {
// isStalled = true;
// coralRollers.setState(CoralRollersState.OUTPUT);
// } else if (!coralRollers.isStalled() && isStalled) {
// isStalled = false;
// coralRollers.setState(CoralRollersState.INTAKING);
// }
// }

// public boolean isFinished() {
// AlignVision.isCollecting = false;
// return coralRollers.clawFrontSensorTriggered();
// }

// }

package frc.robot.Commands.complex.collectCoral;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ControlMap;
import frc.robot.Subsystems.CoralRollers.CoralRollers;
import frc.robot.Subsystems.Vision.AlignVision;
import frc.robot.Subsystems.Wrist.Wrist;
import frc.robot.Subsystems.Wrist.WristStates;
import frc.robot.util.MathUtil;

public class WaitForClawFrontSensor extends Command {
    CoralRollers coralRollers;

    public WaitForClawFrontSensor() {
        coralRollers = CoralRollers.getInstance();
    }

    @Override
    public void execute() {
        if (ControlMap.CO_DRIVER_BUTTONS.button(18).getAsBoolean()) {
            if (MathUtil.isWithinTolerance(Wrist.getInstance().getPosition(), WristStates.INTAKE.wristPosition, 0.015) &&
                    Wrist.getInstance().getState() == WristStates.INTAKE) {
                Wrist.getInstance().setState(WristStates.ELEPHANTIASIS, ClosedLoopSlot.kSlot0);
            } else if (MathUtil.isWithinTolerance(Wrist.getInstance().getPosition(), WristStates.ELEPHANTIASIS.wristPosition, 0.015) &&
                    Wrist.getInstance().getState() == WristStates.ELEPHANTIASIS) {
                Wrist.getInstance().setState(WristStates.INTAKE, ClosedLoopSlot.kSlot0);
            }
        }
    }

    public boolean isFinished() {
        AlignVision.isCollecting = false;
        return coralRollers.clawFrontSensorTriggered();
    }

}