package frc.robot;

import java.lang.Math;

public class Constants {
    public static class DriveConstants {
        public static final double robotLength = 0.60;
        public static final double robotWidth = 0.47;
        // Front Left, Front Right, Back Left, Back Right
        public static final double[] startingPositions = {
            2.155238650739193, 
            -1.003221407532692, 
            2.47277, 
            -2.94421 + Math.PI
        };

        public static final double deadBand = 0.15;

        public static final double neoTurningGearRatio = 12.8;
        public static final double neoTurningResoultion = 42.0;

        public static final double radiansPerEncoderRev = (2 * Math.PI) / neoTurningGearRatio;
    }
}
