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
    private final double IN_TO_TICKS = -246/13.5;

    //distances of arm's pivot point from turret axis when extension is fully retracted and fully extended
    private final double MIN_LENGTH = 10.75;
    private final double MAX_LENGTH = 24.25;
  
    private final double PIVOT_HEIGHT = 0;
    private final double PIVOT_DISTANCE_TO_TIP = 0;
    private final double MOTOR_OFFSET_X = 0;
    private final double MOTOR_OFFSET_Y = 0;
  
    private final double LINK_1 = 0;
    private final double LINK_2 = 0;

    private Vector2 latestDirection = new Vector2();

    private DcMotorEx motor;
    private DigitalChannel frontLimitSwitch;
    private DigitalChannel backLimitSwitch;
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
   
    public LinkageExtension725(HardwareMap hm) {
        init(hm);
        controller = new PIDController(pCoef, iCoef, dCoef);
    }
    public LinkageExtension725(HardwareMap hm, double P, double I, double D){
        init(hm);
        controller.setPID(P,I,D);
    }
    private void init(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "linkage");
        frontLimitSwitch = hm.get(DigitalChannel.class, "front_limit");
        backLimitSwitch = hm.get(DigitalChannel.class, "back_limit");
    }
    public void StartHome() {
        //copied from turret class
        homePower = -homePower;

        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }
    //ALWAYS SET THE OFFSET BEFORE MOVING!! Usually, this will be the extended length of the arm
    public void setOffset(double desiredOffset){offset = desiredOffset;}

    public void GoTo(Vector3 targetVector) {
        GoTo(GetTargetLengthIN(targetVector));
    }
    //this function DOES NOT ACCOUNT FOR THE LINKAGE YET. I have code to do this in teleop 
    public void GoTo(double target_length) {
        double x_attach = target_length + MIN_LENGTH - MOTOR_OFFSET_X - PIVOT_DISTANCE_TO_TIP;
		    double y_attach = PIVOT_HEIGHT - MOTOR_OFFSET_Y;
		
		    double motor_angle = Math.abs(Math.asin((Math.pow(x_attach,2) + Math.pow(y_attach,2) + Math.pow(LINK_1,2) - Math.pow(LINK_2, 2)) / (2 * LINK_1 * Math.sqrt(Math.pow(x_attach ,2) + Math.pow(y_attach, 2))) - Math.atan2(x_attach, y_attach)));
        //TODO: CONFIRM that the encoder reads 0 at motor_angle = 0
        //if not, the stowed angle can be found using this equation
        //Math.atan2(PIVOT_HEIGHT, MIN_LENGTH - PIVOT_DISTANCE_TO_TIP)+Math.acos((Math.pow(LINK_1,2)+Math.pow(PIVOT_HEIGHT,2)+Math.pow(MIN_LENGTH - PIVOT_DISTANCE_TO_TIP,2)-Math.pow(LINK_2,2))/2/(MIN_LENGTH - PIVOT_DISTANCE_TO_TIP)/LINK_1);
        this.targetPosition = Math.toDegrees(motor_angle)*IN_TO_TICKS;
        isBusy = true;
    }
    public Vector2 getVectorTarget(Vector3 targetVector){ // No function for GetTargetLength
        latestDirection.setFromPolar(GetTargetLengthIN(targetVector)+MIN_LENGTH, Math.atan2(targetVector.y,targetVector.x));
        return latestDirection;
    }
    public double GetTargetLengthIN(Vector3 targetVector){
        return Util.clamp(MIN_LENGTH, targetVector.toVector2().magnitude() - offset,MAX_LENGTH)-MIN_LENGTH;
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

        atHome = !backLimitSwitch.getState();
        if (atHome) homed = true;
	if (isHoming && homed) {Stop(); ResetEncoder(); }

        if (isBusy) {
            if (isHoming && !homed) {
                    rawSet(homePower);
                    if (homeTime.seconds() > 3) StartHome();
            } else {
                controller.setPID(pCoef, iCoef, dCoef);
                rawSet(controller.calculate(motor.getCurrentPosition(), targetPosition));
            }
        }


        if (InError() && AUTOSTOP) isBusy = false;
    }
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
}
