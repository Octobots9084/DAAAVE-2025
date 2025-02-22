package frc.robot;

public class States {
    public enum ReefTargetLevel {
        L1,
        L2,
        L3,
        L4,
        BOTTOMALGAE,
        TOPALGAE
    }

    public enum ReefTargetSide {
        LEFT,
        RIGHT,
        ALGAE
    }

    public enum ReefTargetOrientation {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL,
        NONE
    }

    public enum AlignOffset {
        A1(0, 0),
        A2(0, 0),
        A3(0, 0),
        B1(0, 0),
        B2(0, 0),
        B3(0, 0),
        C1(0, 0),
        C2(0, 0),
        C3(0, 0),
        D1(0, 0),
        D2(0, 0),
        D3(0, 0),
        E1(0, 0),
        E2(0, 0),
        E3(0, 0),
        F1(0, 0),
        F2(0, 0),
        F3(0, 0),
        G1(0, 0),
        G2(0, 0),
        G3(0, 0),
        H1(0, 0),
        H2(0, 0),
        H3(0, 0),
        I1(0, 0),
        I2(0, 0),
        I3(0, 0),
        J1(0, 0),
        J2(0, 0),
        J3(0, 0),
        K1(0, 0),
        K2(0, 0),
        K3(0, 0),
        L1(0, 0),
        L2(0, 0),
        L3(0, 0);

        private final int blueOffsetValue;
        private final int redOffsetValue;

        AlignOffset(int blueOffsetValue, int redOffsetValue) {
            this.blueOffsetValue = blueOffsetValue;
            this.redOffsetValue = redOffsetValue;
        }

        public int getBlueOffsetValue() {
            return blueOffsetValue;
        }

        public int getRedOffsetValue() {
            return redOffsetValue;
        }
    }

    public enum AlignState {
        Reef,
        Processor,
        Source,
        Manual
    }
}
