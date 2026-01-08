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

//TODO 7/15/25 - Frank
// - set constants
// - figure out at what angle the encoder reads 0
// - integrate with arm class

public class LinkageExtension725 {
  
    public static final double PID_P_DEFAULT = 2.5;
    public static final double PID_I_DEFAULT = 0;
    public static final double PID_D_DEFAULT = 0.1;
    private double pCoef = PID_P_DEFAULT, iCoef = PID_I_DEFAULT, dCoef = PID_D_DEFAULT;
    private static final double EXTENSION_MIN_POSITION = 0;
    private static final double EXTENSION_MAX_POSITION = 10;
    private static final double IN_TO_TICKS = 1725/EXTENSION_MAX_POSITION;
    private static final double EXTENSION_MIN_POSITION_TICKS = EXTENSION_MIN_POSITION* IN_TO_TICKS;
    private static final double EXTENSION_MAX_POSITION_TICKS = EXTENSION_MAX_POSITION * IN_TO_TICKS;

    private Vector2 latestDirection = new Vector2();

    private DcMotorEx motor;
    private DigitalChannel backLimitSwitch;
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    //value copied from Turret class
    private double homePower = -0.5;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition = EXTENSION_MIN_POSITION;
    private double currentPosition;
    private double currentPower = 0;

    //desired distance offset from the given target in inches
    //this variable can be used to set the extension distance after setting the arm's rotation
    private double offset;

    private PIDController controller;

    public boolean AUTOSTOP = true;
   
    public LinkageExtension725(HardwareMap hm) {
        init(hm);
        controller = new PIDController(pCoef, iCoef, dCoef);
    }
    public LinkageExtension725(HardwareMap hm, double P, double I, double D){
        init(hm);
        controller.setPID(P,I,D);
    }
    private void init(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "extension");
        backLimitSwitch = hm.get(DigitalChannel.class, "rear_limit");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void StartHome() {
        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }
    //ALWAYS SET THE OFFSET BEFORE MOVING!! Usually, this will be the extended length of the arm
    public void setOffset(double desiredOffset){offset = desiredOffset;}

    public void GoTo(double target_length) {
        if(target_length>10){target_length = 10;}
        if(target_length<0){target_length = 0;}
        this.targetPosition = target_length*IN_TO_TICKS;
        isBusy = true;
    }

    public double GetPos() {return GetRawPos() / IN_TO_TICKS;}
    public boolean isAtHome() {return atHome;}
    public double GetRawPos() {return currentPosition;}
    public double GetTargetPos() {return targetPosition / IN_TO_TICKS;}
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
        currentPower = 0;
        rawSet(0);
        isBusy = false;
        isHoming = false;
    }
    public void ResetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Update() {

        atHome = !backLimitSwitch.getState();
        currentPosition = motor.getCurrentPosition();
        if (atHome) homed = true;
	    if (isHoming && homed) {Stop(); ResetEncoder(); }

        if (isBusy) {
            if (isHoming && !homed) {
                    currentPower = homePower;
                    if (homeTime.seconds() > 3) StartHome();
            } else {
                controller.setPID(pCoef, iCoef, dCoef);
                currentPower = controller.calculate(currentPosition, targetPosition);
                if(Math.abs(currentPosition - targetPosition) < 10){
                    isBusy = false;
                    currentPower = 0;
                }
            }
        }
        //safety lockouts
        if(!isHoming){
            if (atHome) {
                currentPower = Math.min(Math.max(currentPower, 0), 1);
            }
            if (currentPosition >= EXTENSION_MAX_POSITION_TICKS) {
                currentPower = Math.min(Math.max(currentPower, -1), 0);
            }
        }
        rawSet(currentPower);

    }
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
}
