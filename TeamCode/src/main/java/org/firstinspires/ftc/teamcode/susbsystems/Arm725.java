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

//TODO 7/22/25 - Frank
// - set constants
// - Add potentiometer / encoder code and add arm PID

public class Arm725 {
  
    private final double DEG_TO_TICKS = 0;
  
    private final double PIVOT_HEIGHT = 0;
  
    private final double PIVOT_TO_WRIST = 0;

    private final double STOWED_ANGLE_DEG = 0;

    private Vector2 latestDirection = new Vector2();
  
    private DcMotorEx motor;

    private AnalogInput pot1;
  
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    //value copied from Turret class
    private double homePower = 0.3;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition;

    //desired distance offset from the given target in inches
    //this variable can be used to set the extension distance after setting the arm's rotation
    private double offset;

    private PIDController controller;

    public boolean AUTOSTOP = true;

    public LinkageExtension(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "linkage");
        frontLimitSwitch = hm.get(DigitalChannel.class, "front_limit");
        backLimitSwitch = hm.get(DigitalChannel.class, "front_limit");
        controller = new PIDController(Constants.EXTENSION_P, Constants.EXTENSION_I, Constants.EXTENSION_D);
        pot1 = hardwareMap.get(AnalogInput.class, "pot1");
    }
    public void StartHome() {
        //copied from turret class
        homePower = -homePower;

        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }
    //ALWAYS SET THE OFFSET BEFORE MOVING!!
    public void setOffset(double desiredOffset){offset = desiredOffset;}

    public void GoTo(Vector3 targetVector) {
        GoTo(targetVector.z);
    }
    //this function DOES NOT ACCOUNT FOR THE LINKAGE YET. I have code to do this in teleop 
    public void GoTo(double target_height) {
        
        //TODO: CONFIRM that the encoder reads 0 at motor_angle = 0
        //if not, the stowed angle can be found using this equation
        //Math.atan2(PIVOT_HEIGHT, MIN_LENGTH - PIVOT_DISTANCE_TO_TIP)+Math.acos((Math.pow(LINK_1,2)+Math.pow(PIVOT_HEIGHT,2)+Math.pow(MIN_LENGTH - PIVOT_DISTANCE_TO_TIP,2)-Math.pow(LINK_2,2))/2/(MIN_LENGTH - PIVOT_DISTANCE_TO_TIP)/LINK_1);
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
    public double getTargetAngleDeg(Vector3 targetVector){
        return Math.toDegrees(getTargetAngleRad());
    }
    public double GetPos() {return GetRawPos() / IN_TO_TICKS;}
    public double GetRawPos() {return motor.getCurrentPosition();}
    public double GetTargetPos() {return targetPosition / IN_TO_TICKS;}
    public double GetRawTargetPos() {return targetPosition;}
    public boolean InError() { return Math.abs(GetRawPos() - GetRawTargetPos()) < Constants.TURRET_ERROR/2;}
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
                controller.setPID(Constants.EXTENSION_P, Constants.EXTENSION_I, Constants.EXTENSION_D);
                rawSet(controller.calculate(motor.getCurrentPosition(), targetPosition));
            }
        }


        if (InError() && AUTOSTOP) isBusy = false;
    }
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
}
