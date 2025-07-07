package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Util;

public class Drive { // Working but Could be Missing Some Needed Functions
    /**
     *      Usage Example:
     * Move Forward for One Second, Turn for One Second, and then Turn Back While Moving Forward for Two Seconds
     *
     * drive.Set(1);
     * sleep(1000);
     * drive.Set(1,1);
     * sleep(1000);
     * drive.Set(1,-0.5);
     * sleep(2000);
     *
    */
    private CRServo leftMotor, rightMotor;
    private double minPower = 0.2, maxPower = 0.3;
    private boolean isMoving;
    public Drive(HardwareMap hm) {
        leftMotor = hm.get(CRServo.class, "leftdrive");
        rightMotor = hm.get(CRServo.class, "rightdrive");
        leftMotor.setDirection(CRServo.Direction.REVERSE);
    }
    private double MotorPower(double power) {
        return Util.rangeZero(Util.IntClamp(power),minPower,maxPower);
    }
    public void Set(double power) {
        double motorPower = MotorPower(power);
        leftMotor.setPower(motorPower); rightMotor.setPower(motorPower);
    }
    public void Set(double power, double steering) {
        // If Steering is -1 Pivot around Center Left, if Steering is 1 Pivot around Center Right
        // If Steering is -0.5 Steer Robot to left while moving forward, and vice versa for 0.5
        // The Reason for using Steering is it will work better for correction implementation as you can use the steering input for where you would put your correction factor like yaw error for example
        steering = Util.IntClamp(steering);
        double leftSteering=1, rightSteering=1;
        double calculatedSteering = -(Math.abs(steering)-0.5)*2;
        if (steering < 0) leftSteering = calculatedSteering;
        else rightSteering = calculatedSteering;

        double motorPower = MotorPower(power);
        leftMotor.setPower(motorPower*leftSteering); rightMotor.setPower(motorPower*rightSteering);
    }
    public void StartMoving() { isMoving = true; Set(Constants.DRIVE_FORWARD_POWER); }
    public void StopMoving() { isMoving = false; Set(0); }
    public void CorrectAngleBy(double correctionFactor) {
        if (isMoving) {
            Set(Constants.DRIVE_FORWARD_POWER, correctionFactor);
        }
    }
    public double GetLeftPower() {return leftMotor.getPower();}
    public double GetRightPower() {return rightMotor.getPower();}
}
