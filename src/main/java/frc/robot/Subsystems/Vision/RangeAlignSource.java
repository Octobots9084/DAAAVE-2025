package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeAlignSource extends SubsystemBase {
    CANrange backRange;
    PIDController backRangePID;

    private static RangeAlignSource instance;

    public static RangeAlignSource getInstance() {
        if (instance == null) {
            instance = new RangeAlignSource();
        }
        return instance;
    }

    public RangeAlignSource() {
        this.backRange = new CANrange(0, "KrakensBus");
        this.backRangePID = new PIDController(2, 0, 0);
    }

    public double getBackRange() {
        return backRange.getDistance().getValueAsDouble();
    }

    public ChassisSpeeds getAlignChassisSpeeds() {
        return new ChassisSpeeds(backRangePID.calculate(this.getBackRange()), 0, 0);
    }
}
