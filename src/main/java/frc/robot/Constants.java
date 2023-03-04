package frc.robot;

import java.lang.Math;

public class Constants {
    public static class DriveConstants {
        public static final double robotLength = 0.60;
        public static final double robotWidth = 0.47;
        // Front Left, Front Right, Back Left, Back Right
        public static final double[] startingPositions = {
            -0.349746912717819 + Math.PI, 
            -3.060285486280918, 
            2.66912117600441, 
            -2.439024522900581
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
 
        public static final float armDownHardLimit = 5;

        // public static final double driveLimit = 16.6;
        public static final double driveLimit = 5.0;
        public static final double groundPosition = 25.690353393554688;
        public static final double middlePosition = 110.78571081161499;
        public static final double highPosition = 125.78571081161499;
        public static final double humanPosition = 120.78571081161499;

        public static final int deadband = 3;

        public static final double craneUpP = 0.01;
        public static final double craneDownP = 0.005;
        
        // TODO Change Limits
        public static final double extensionLimit = 9.0;
        public static final double retractionLimit = 1.0;

        public static final double extensionP = 0.01;

        public static final double extendGround = 8.7;
        public static final double extendMiddle = -10.6191520690918;
        public static final double extendHigh = 8.0;
        public static final double extendHuman = 9.0;
    }
}
