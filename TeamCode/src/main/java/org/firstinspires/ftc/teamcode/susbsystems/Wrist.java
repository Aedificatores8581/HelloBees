package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Util;

//Todo
// Servo and Linkage are non-linear so servo range is 0 degrees -> 90 degrees with .2 being 0 and 1 being 90 degrees
// This makes height & angle not accurate at lower values but it seems good enough and i suspect usage this won't matter
// as I think wrist should go to a standard position in space
// but the todo is linkage math to make the telemetry work better

public class Wrist {

    //Wrist Constants
    private static final double DEG_TO_SERVO_POS = 90;
    private static final double WRIST_RADIUS = 7.5;
    private static final double WRIST_MAX_HEIGHT = 7.5;
    private static final double WRIST_MIN_HEIGHT = 4;
    private static final double WRIST_MAX_POSITION = 1;
    private static final double WRIST_MIN_POSITION = .2;

    //Shoulder private variables
    private Servo wrist_servo;
     private boolean wrist_is_busy = false;
    private double targetPosition = 0;
    private double currentPosition;
    private double currentPos = 0.5;
    private double targetHeight = 0;
    private static ElapsedTime wristStopwatch = new ElapsedTime();

    public Wrist(HardwareMap hm) {
        init(hm);
    }
    private void init(HardwareMap hm) {
        wrist_servo = hm.get(Servo .class, "wrist");
    }

    public void GoToHeight(double target_height) {
        double temp;
        if(target_height > WRIST_MAX_HEIGHT){target_height = WRIST_MAX_HEIGHT;}
        if(target_height < WRIST_MIN_HEIGHT){target_height = WRIST_MIN_HEIGHT;}
        targetHeight = target_height;
        temp = target_height / WRIST_RADIUS;
        temp = Math.asin(temp);
        temp = Math.toDegrees(temp);
        targetPosition = temp/DEG_TO_SERVO_POS;
        this.SetPos(temp/DEG_TO_SERVO_POS);
        wrist_is_busy = true;
        wristStopwatch.reset();
    }
    public void GoToAngle(double target_angle) {

        this.targetPosition =  target_angle/DEG_TO_SERVO_POS;
        this.SetPos(targetPosition);
        wrist_is_busy = true;
        wristStopwatch.reset();
    }
    public double GetPos() {return currentPosition * DEG_TO_SERVO_POS;}
    public double GetHeight() {
        double temp;
        temp = currentPosition * DEG_TO_SERVO_POS;
        temp = Math.toRadians(temp);
        temp = Math.sin(temp);
        return (WRIST_RADIUS *temp);
    }
    public double GetRawPos() {return currentPosition;}
    public double GetTargetPos() {return targetPosition * DEG_TO_SERVO_POS;}
    public double GetRawTargetPos() {return targetPosition;}
    public double GetTargetHeight() {return targetHeight;}
    public boolean IsBusy() {return wrist_is_busy;}

    public void SetPos(double power) {
        wrist_is_busy = false;
        //safety checks
        power = Math.min(Math.max(power, WRIST_MIN_POSITION), WRIST_MAX_POSITION);
        wrist_servo.setPosition(power);
    }
    public void Stop() {
        wrist_is_busy = false;
    }

    public void Update() {
        currentPosition = wrist_servo.getPosition();
        if (wrist_is_busy && wristStopwatch.milliseconds() > 300) {
            wrist_is_busy = false;
        } else if (!wrist_is_busy) {
            wristStopwatch.reset();
        }
    }
}

