package frc.robot;

import java.lang.Math;

public class Constants {
    public static class DriveConstants {
        public static final double robotLength = 0.60;
        public static final double robotWidth = 0.47;
        // Front Left, Front Right, Back Left, Back Right
        public static final double[] startingPositions = {
            2.785703480243683, 
            0.148795835673809 + Math.PI, 
            -0.467863194644451 + Math.PI, 
            0.691823936998844 + Math.PI
        };

        public static final double deadBand = 0.15;

        public static final double neoTurningGearRatio = 12.8;
        public static final double neoTurningResoultion = 42.0;

        public static final double radiansPerEncoderRev = (2 * Math.PI) / neoTurningGearRatio;
    }
}
