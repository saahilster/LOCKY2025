// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

/** Add your docs here. */
public class Constants {

    public static final class ControllerConstants { // for playstation 5 controller
        public static final int driver = 0,
                operator = 1,
                b_SQR = 1,
                b_X = 2,
                b_O = 3,
                b_TRI = 4,
                b_L1 = 5,
                b_R1 = 6,
                b_L2 = 7,
                b_R2 = 8,
                b_PIC = 9,
                b_MEN = 10,
                b_LJOY = 11,
                b_RJOY = 12,
                b_LOG = 13,
                b_PAD = 14,
                b_MIC = 15,

                // PURPOSE: arm moves based upon pressure on triggers
                leftTriggerAxis = 3,
                rightTriggerAxis = 4;
    }

    public static final class AutonConstants {
        // public static double drivekP = 0.5,
        // drivekI = 0.0,
        // drivekD = 0.1,

        // steerkP = 0.5,
        // steerkI = 0.0,
        // steerkD = 0.1;

        // Translational Gains (XY)
        public static double xP = 1.1, xI = 0, xD = 0;

        // Angular Gains (Theta)
        public static double tP = 4.1, tI = 0.85, tD = 0;

        // // Translational Gains (XY)
        // public static double xP = 0.5, xI = 2.25, xD = 0.1;

        // // Angular Gains (Theta)
        // public static double tP = 0.55, tI = 4, tD = 0.2;
    }

    public static final class MotorConstants {
        public static final int leftCascade= 0,
        rightCascade = 0,
        pivot = 0,
        algaeIntake = 0,
        wrist = 0,
        coralIntake = 0;
    }
}
