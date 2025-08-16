package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    public VisionIO io;
    VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public static VisionSubsystem instance;

    public static VisionSubsystem getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Vision instance not set");
        }
        return instance;
    }

    public static VisionSubsystem setInstance(VisionIO io) {
        instance = new VisionSubsystem(io);
        return instance;
    }

    public VisionSubsystem(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);
        if (VisionConstants.USE_VISION) {
            io.updatePose();

        } else {
            io.closeNotifiers();
        }
    }


}
