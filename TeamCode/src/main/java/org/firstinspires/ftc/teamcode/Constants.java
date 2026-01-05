package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Math.Vector3;

@Config
public class Constants {
    // Drive Constants
    public static double DRIVE_FORWARD_POWER = 1;

    public static double X_CAM = -10, Y_CAM = 0, Z_CAM= -1.5;
    public static double YAW_CAM = Math.PI/2, PITCH_CAM = 0, ROLL_CAM = 0;
    public static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS,YAW_CAM, PITCH_CAM, ROLL_CAM, 0);


    //TODO: we should consider moving these values into the turret, arm and extension classes before building the redesign
    public static double TURRET_P =0.001, TURRET_I =0, TURRET_D = 0.00003;
    public static double ARM_P = 2.5, ARM_I = 0, ARM_D = .1;
    public static double ARM_ERROR = 0;
    public static double EXTENSION_P = 2.5, EXTENSION_I = 0, EXTENSION_D = .1;
    //the unit vector that points in the direction of the arm's home position 
    public Vector3 coordinateHome = new Vector3(1,0,0);
    public static final double ZERO_DEGREES = .315;
    public static final double FORTYFIVE_DEGREES = .279;
    public static final double NINETY_DEGREES = .255;
    public static final double ONETHIRTYFIVE_DEGREES = .241;
    public static final double ONEEIGHTY_DEGREES = .216;
    public static final double TWOTWENTYFIVE_DEGREES = .195;

    // turret min/max  analog potentiometer values
    public static final double TURRET_MIN_POSITION = .185;
    public static final double TURRET_MAX_POSITION = .315;

    public static final double TURRET_ERROR = .001;
}
