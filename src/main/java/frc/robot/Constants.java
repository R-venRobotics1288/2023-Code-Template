package frc.robot;

import java.lang.Math;

public class Constants {
    public static class DriveConstants {
        public static final double robotLength = 0.60;
        public static final double robotWidth = 0.47;
        // Front Left, Front Right, Back Left, Back Right
        public static final double[] startingPositions = {
            0.023009665310383, 
            0.662678360939026, 
            1.222580216825008, 
            -1.533977687358856
        };

        public static final double deadBand = 0.15;

        public static final double neoTurningGearRatio = 12.8;
        public static final double neoTurningResoultion = 4096.0;

        public static final double radiansPerEncoderTick = (2 * Math.PI) / (neoTurningGearRatio * neoTurningResoultion);
    }
}
