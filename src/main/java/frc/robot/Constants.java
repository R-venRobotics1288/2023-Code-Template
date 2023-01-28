package main.java.frc.robot;

import java.lang.Math;

public class Constants {
    public static class DriveConstants {
        public static final double robotLength = 0.60;
        public static final double robotWidth = 0.47;
        // Front Left, Front Right, Back Left, Back Right
        public static final double[] offsets = {
            -3.963 + Math.PI, 
            6.263 + Math.PI, 
            5.022 + Math.PI, 
            5.100 + Math.PI
        };

        public static final double deadBand = 0.15;
    }
}
