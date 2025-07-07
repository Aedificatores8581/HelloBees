package org.firstinspires.ftc.teamcode.util;

public class Util {
    // A place to put any simple custom math function that might be re-used and will also save time while coding
    public static double IntClamp(double input) {
        if (input < -1) return -1;
        else if (input > 1) return 1;
        else return input;
    }
    public static int pmz (double input) {
        // returns -1, 0, or 1 depending on if the number was less than, equal to, or greater than 0
        return (int)IntClamp(Math.ceil(input));
    }
    public static double range (double input, double min, double max) {
        // Returns a number in-between the minimum and maximum based on an input 0-1
        // For example, if the input was 1 and the max was 0.5 the return would be 0.5
        // Another example would be if the input was 0.5, min was 0.2, and max was 0.3 the return would be 0.25
        return (input*(max-min))+(min*pmz(input));
    }
    public static double rangeZero (double input, double min, double max) {
        // Does the same thing as range except if the input is equal to zero it will set the output to zero
        if (input == 0) return 0;
        else return range(input, min, max);
    }

}
