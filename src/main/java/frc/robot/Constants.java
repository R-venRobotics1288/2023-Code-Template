package frc.robot;

import java.lang.Math;

public class Constants {
    public static class DriveConstants {
        public static final double robotLength = 0.60;
        public static final double robotWidth = 0.47;
        // Front Left, Front Right, Back Left, Back Right
        public static final double[] startingPositions = {
            2.793373368680477, 
            0.131922081112862 - Math.PI, 
            1.066114492714405, 
            -2.428286679089069
        };

        public static final double turningP = 0.6;
        public static final double drivingP = 0.25;

        public static final double deadBand = 0.15;

        public static final double neoTurningGearRatio = 12.8;
        public static final double neoTurningResoultion = 42.0;

        public static final double radiansPerEncoderRev = (2 * Math.PI) / neoTurningGearRatio;
    }
    
    public static class ArmConstants {
        public static final float downSoftLimit = 19.5f;
        public static final float upSoftLimit = 60.5f;

        public static final double groundPosition = 16.6;
        public static final double middlePosition = 34.9;
        public static final double highPosition = 100.1;
        public static final double humanPosition = 47.0;

        public static final int deadband = 3;

        public static final double craneP = 0.01;
        
        // TODO Change Limits
        public static final double extensionLimit = 9.0;
        public static final double retractionLimit = 10.0;

        public static final double extensionP = 0.01;

        public static final double extendGround = 8.7;
        public static final double extendMiddle = 9.89;
        public static final double extendHigh = 8.0;
        public static final double extendHuman = 9.0;
    }
}
