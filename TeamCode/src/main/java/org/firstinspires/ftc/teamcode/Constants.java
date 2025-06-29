package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public class Constants {


    final public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,-90, -90, 0, 0);
    final public static double X_CAM = -11, Y_CAM = 0, Z_CAM= -1.5;

    public static double turret_p =0.001, turret_i =0, turret_d = 0.00003;
    public static double shoulder_p = 2.5, shoulder_i = 0, shoulder_d = .1;
}
