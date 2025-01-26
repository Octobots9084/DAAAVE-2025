package frc.robot;

public class States {
  public enum ReefTargetLevel {
    L1,
    L2,
    L3,
    L4
  }

  public enum ReefTargetSide {
    LEFT,
    RIGHT
  }

  public enum ReefTargetOrientation {
    AB,
    CD,
    EF,
    GH,
    IJ,
    KL

  }

  public enum AlignOffset {
    A1(1, 1),
    A2(2, 2),
    A3(3, 3),
    B1(4, 4),
    B2(5, 5),
    B3(6, 6),
    C1(7, 7),
    C2(8, 8),
    C3(9, 9),
    D1(10, 10),
    D2(11, 11),
    D3(12, 12),
    E1(13, 13),
    E2(14, 14),
    E3(15, 15),
    F1(16, 16),
    F2(17, 17),
    F3(18, 18),
    G1(19, 19),
    G2(20, 20),
    G3(21, 21),
    H1(22, 22),
    H2(23, 23),
    H3(24, 24),
    I1(25, 25),
    I2(26, 26),
    I3(27, 27),
    J1(28, 28),
    J2(29, 29),
    J3(30, 30),
    K1(31, 31),
    K2(32, 32),
    K3(33, 33),
    L1(34, 34),
    L2(35, 35),
    L3(36, 36);

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
