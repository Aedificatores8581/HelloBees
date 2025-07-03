package org.firstinspires.ftc.teamcode.susbsystems;

public class Util {
    // A place to put any simple custom math function that might be re-used and will also save time while coding
    public static double IntClamp(double input) {
        if (input < -1) return -1;
        else if (input > 1) return 1;
        else return input;
    }
}
