package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Constants {
    // Drive Constants
    public static double DRIVE_FORWARD_POWER = 1;

    public static double X_CAM = -44/*Actual Number with Less Accurate Results: 11???*/, Y_CAM = 0, Z_CAM= -1.5;
    public static double YAW_CAM = Math.PI/2, PITCH_CAM = 0, ROLL_CAM = 0;
    public static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.RADIANS,YAW_CAM, PITCH_CAM, ROLL_CAM, 0);

    public static double turret_p =0.001, turret_i =0, turret_d = 0.00003;
    public static double shoulder_p = 2.5, shoulder_i = 0, shoulder_d = .1;
}
