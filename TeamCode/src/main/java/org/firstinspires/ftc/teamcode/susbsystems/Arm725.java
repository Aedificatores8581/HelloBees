package org.firstinspires.ftc.teamcode.susbsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Math.Vector2;
import org.firstinspires.ftc.teamcode.util.Math.Vector3;
import org.firstinspires.ftc.teamcode.util.Util;

//TODO 7/23/25 - Frank
// - set constants
// - Add error and PID constants to Constants.java

public class Arm725 {
    public static final double PID_P_DEFAULT = 2.5;
    public static final double PID_I_DEFAULT = 0;
    public static final double PID_D_DEFAULT = .1;
  
    private static final double DEG_TO_TICKS = 0.0122222; // copied from teleop, needs re-measured

    private static final double HOME_ANGLE = 0;
    private static final double HOME_Height = 0;

    // the value in volts the potentiometer reads at stowed position
    private final double POT_STOWED_READING = 0.495;
    // the value in volts the potentiometer reads when the arm points straight ahead
    private final double POT_0DEG_READING = 1.32;
    //private final double DEG_TO_VOLT = (POT_STOWED_READING - POT_0DEG_READING) / STOWED_ANGLE_DEG;

    private static final double PIVOT_TO_WRIST = 14;

    //private Vector2 latestDirection = new Vector2();
  
    private DcMotorEx motor;

    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    //value copied from Turret class
    private double homePower = 0.3;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition;
    private double currentPosition;
    private double currentPower = 0;
    private double pCoef = PID_P_DEFAULT, iCoef = PID_I_DEFAULT, dCoef = PID_D_DEFAULT;
    //desired distance offset from the given target in inches
    //this variable can be used to set the extension distance after setting the arm's rotation

    private PIDController controller;

    public boolean AUTOSTOP = true;

    public Arm725(HardwareMap hm) {
        init(hm);
    }
    public Arm725(HardwareMap hm, double P, double I, double D){
        init(hm);
        setPID(P,I,D);
    }
    private void init(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "shouldergobilda");
        controller = new PIDController(pCoef, iCoef, dCoef);
        //pot1 = hm.get(AnalogInput.class, "pot1");
    }
    public void setPID(double P, double I, double D){
        pCoef = P; iCoef = I; dCoef = D;
    }
    public void StartHome() {
        ResetEncoder();
        homed = true;
        homeTime.reset();
    }
    //public void GoTo(Vector3 targetVector) {GoTo(targetVector.z);}
    public void GoToHeight(double target_height) {
        
        this.targetPosition = Math.toDegrees(Math.asin(target_height/PIVOT_TO_WRIST)*DEG_TO_TICKS);
        isBusy = true;
    }
    public void GoToAngle(double target_angle) {

        this.targetPosition = Math.toDegrees(Math.asin(target_angle/PIVOT_TO_WRIST)*DEG_TO_TICKS);
        isBusy = true;
    }
    //public Vector2 getVectorTarget(Vector3 targetVector){ // No function for GetTargetLength
    //    latestDirection.setFromPolar(PIVOT_TO_WRIST, getTargetAngleRad(targetVector));
    //    return latestDirection;
    //}
    public double getTargetAngleRad(Vector3 targetVector){
        return Util.clamp(Math.PI/-2, Math.atan2(targetVector.z,targetVector.toVector2().magnitude()),Math.PI/2);
    }
    public double getTargetAngleDeg(Vector3 targetVector){
        return Math.toDegrees(getTargetAngleRad(targetVector));
    }
    public double GetPos() {return currentPosition / DEG_TO_TICKS;}
    public double GetHeight() {return currentPosition;}
    public double GetRawPos() {return currentPosition;}
    public double GetTargetPos() {return (targetPosition) / DEG_TO_TICKS;}
    public double GetRawTargetPos() {return targetPosition;}
    public boolean IsBusy() {return isBusy;}
    private void rawSet(double power) {
        motor.setPower(Util.IntClamp(power));
    }
    public void SetPower(double power) {
        isBusy = false;
        currentPower = power;
    }
    public void Stop() {
        rawSet(0);
        currentPower = 0;
        isBusy = false;
        isHoming = false;
    }
    public void ResetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Update() {

        currentPosition = motor.getCurrentPosition();
        if (isBusy) {
           controller.setPID(pCoef, iCoef, dCoef);
           currentPower = controller.calculate(GetPos(), targetPosition);
        }
        //add safety checks
        rawSet(currentPower);
    }
    public double GetPower() {return currentPower;}
    public boolean Homed() {return homed;}
}
