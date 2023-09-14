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
            0.776192709803581 + Math.PI,
            -2.439024522900581
        };

        public static final double turningP = 0.6;
        public static final double drivingP = 0.25;

        public static final double deadBand = 0.1;

        public static final double neoTurningGearRatio = 12.8;
        public static final double neoTurningResoultion = 42.0;

        public static final double radiansPerEncoderRev = (2 * Math.PI) / neoTurningGearRatio;

        public static final double speedRateLimit = 1.0;
        public static final double rotRateLimit = 3;

        // Up double list for x and y speeds
        public static final double[] upJoytickInputs = {0.0, -1.0};
        // Left double list for x and y speeds
        public static final double[] leftJoytickInputs = {-1.0, 0};
        // Right double list for x and y speeds
        public static final double[] rightJoytickInputs = {1.0, 0.0};
        // Down double list for x and y speeds
        public static final double[] downJoytickInputs = {0.0, 1.0};
    }
    
    public static class ArmConstants {
 
        public static final double armUpHardLimit = 155;
        public static final double armDownHardLimit = 1;

        // public static final double driveLimit = 16.6;
        public static final double driveLimit = 1.0;
        public static final double groundPosition = 30.690353393554688;
        public static final double middlePosition = 110.78571081161499;
        public static final double highPosition = 135.78571081161499;
        public static final double humanPosition = 105;

        public static final int deadband = 3;

        public static final double craneUpGroundP = 0.015;
        public static final double craneUpMidHighP = 0.01;
        public static final double craneDownP = 0.005;
        
        // TODO Change Limits
        // Neagtive extension, Postive retraction
        public static final double extensionLimit = 370.0;
        public static final double retractionLimit = 3.0;

        public static final double extensionP = 0.01;

        public static final double extendGround = 60.95191192626953;
        public static final double extendMiddle = 85.86058807373047;
        public static final double extendHigh = 365.19281005859375;
        public static final double extendHuman = 9.0;
    }
}
