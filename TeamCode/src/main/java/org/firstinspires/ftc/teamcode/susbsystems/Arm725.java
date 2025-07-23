package org.firstinspires.ftc.teamcode.susbsystems;

import com.arcrobotics.ftclib.controller.PIDController;
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
    public static final double PID_P_DEFAULT = 0;
    public static final double PID_I_DEFAULT = 0;
    public static final double PID_D_DEFAULT = 0;
  
    private final double DEG_TO_TICKS = 0;
    private final double PIVOT_HEIGHT = 0;
    private final double PIVOT_TO_WRIST = 0;
    private final double STOWED_ANGLE_DEG = 0;

    // the value in volts the potentiometer reads at stowed position
    private final double POT_STOWED_READING = 0;
    // the value in volts the potentiometer reads when the arm points straight ahead
    private final double POT_0DEG_READING = 0;
    private final double DEG_TO_VOLT = (POT_STOWED_READING - POT_0DEG_READING) / STOWED_ANGLE_DEG;
    private final double RAD_TO_VOLT = Math.toRadians(DEG_TO_VOLT);

    private Vector2 latestDirection = new Vector2();
  
    private DcMotorEx motor;

    private AnalogInput pot1;
  
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    //value copied from Turret class
    private double homePower = 0.3;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition;
    private double pCoef = PID_P_DEFAULT, iCoef = PID_I_DEFAULT, dCoef = PID_D_DEFAULT;
    //desired distance offset from the given target in inches
    //this variable can be used to set the extension distance after setting the arm's rotation
    private double offset;

    private PIDController controller;

    public boolean AUTOSTOP = true;

    public Arm725(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "arm");
        controller = new PIDController(pCoef, iCoef, dCoef);
        pot1 = hardwareMap.get(AnalogInput.class, "pot1");
    }
    public Arm725(HardwareMap hm, double P, double I, double D){
        setPID(P,I,D);
        this.ARM725(hm);
    }
    public void setPID(double P, doublue I, double D){
        pCoef = P; iCoef = I; dCoef = D;
    }
    public void StartHome() {
        //copied from turret class
        homePower = -homePower;

        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }
    public void GoTo(Vector3 targetVector) {
        GoTo(targetVector.z);
    }
    public void GoTo(double target_height) {
        
        this.targetPosition = Math.toDegrees(Math.asin(target_height/PIVOT_TO_WRIST)*DEG_TO_TICKS);
        isBusy = true;
    }
    public Vector2 getVectorTarget(Vector3 targetVector){ // No function for GetTargetLength
        latestDirection.setFromPolar(PIVOT_TO_WRIST, getTargetAngleRad(targetVector));
        return latestDirection;
    }
    public double getTargetAngleRad(Vector3 targetVector){
        return Util.clamp(Math.PI/-2, Math.atan2(targetVector.z,targetVector.toVector2().getMagnitude()),Math.PI/2);
    }
    public double getTargetAngleDeg(Vector3 targetVector){00
        return Math.toDegrees(getTargetAngleRad());
    }
    public double GetPos() {return GetRawPos() / DEG_TO_VOLT + POT_0DEG_READING;}
    public double GetRawPos() {return pot1.getVoltage();}
    public double GetTargetPos() {return (targetPosition - POT_0DEG_READING) / DEG_TO_VOLT;}
    public double GetRawTargetPos() {return targetPosition;}
    public boolean InError() { return Math.abs(GetRawPos() - GetRawTargetPos()) < Constants.ARM_ERROR/2;}
    public boolean IsBusy() {return isBusy;}
    private void rawSet(double power) {
        motor.setPower(Util.IntClamp(power));
    }
    public void SetPower(double power) {
        isBusy = false;
        rawSet(power);
    }
    public void Stop() {
        rawSet(0);
        isBusy = false;
        isHoming = false;
    }
    public void ResetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Update() {
        //limit switch code copied from turret. Needs to be adjusted for current setup
        //
        //atHome = !homeLimitSwitch.getState();
        //if (atHome) homed = true;
        //if (isHoming && homed) {Stop(); ResetEncoder(); }

        if (isBusy) {
            if (isHoming && !homed) {
                    rawSet(homePower);
                    if (homeTime.seconds() > 3) StartHome();
            } else {
                controller.setPID(pCoeff, iCoef, dCoef);
                rawSet(controller.calculate(GetPos(), targetPosition));
            }
        }


        if (InError() && AUTOSTOP) isBusy = false;
    }
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
}
