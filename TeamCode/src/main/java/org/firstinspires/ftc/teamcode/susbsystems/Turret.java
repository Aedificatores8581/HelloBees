package org.firstinspires.ftc.teamcode.susbsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Util;

//TODO 7/14/25 - Frank
// - confirm default 0 degree position and rotation direction
//    - affects GoTo()
// - detect if a given point is too close to the robot
// - integrate with extension system for a getTargetVector() function
// - latestDirection vector is a clunky implementation. I should streamline it
//    - current solution: add a static method to Vector2 to create a new polar vector
// - move getVectorTarget() to the full system class, as it needs arm and extension information


public class Turret {
    //changed this variable from TICKS_TO_DEG to DEG_TO_TICKS for accuracy
    private final double DEG_TO_TICKS = 5900d/90d;

    private DcMotorEx motor;
    private DigitalChannel homeLimitSwitch;
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    private double homePower = 0.3;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition;
    private double targetPosition

    //represents the direction the turret has last been set to
    public Vector2 latestDirection = new Vector2();
    
    private PIDController controller;

    public boolean AUTOSTOP = true;

    public Turret(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "turret");
        homeLimitSwitch = hm.get(DigitalChannel.class, "turret_home");
        controller = new PIDController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D);
    }
    public void StartHome() {
        homePower = -homePower;
        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }
    public void GoTo(Vector3 targetVector) {
        GoTo(getTargetAngleDeg(targetVector));
    }
    
    public void GoTo(double targetAngle) {
        this.targetPosition = targetAngle*DEG_TO_TICKS;
        isBusy = true;
    }
    public Vector2 getUnitVectorTarget(Vector3 targetVector){
        latestDirection.setFromPolar(1, GetTargetAngleRad(targetVector));
        return latestDirection;
    }
    //This function should go in the full system class
    //public Vector3 getVectorTarget(Vector3 targetVector, double extensionLength){return getUnitVectorTarget(targetVector).toVector3(0).add(0,0,extensionHeight).scalar(extensionLength + retractedLength);}
    
    public double GetTargetAngleRad(Vector3 targetVector){
        return Math.atan2(targetVector.y,targetVector.x,0);
    }
    public double GetTargetAngleDeg(Vector3 targetVector){ return Math.toDegrees(GetTargetAngleRad(targetVector));}
    public void GetTargetPosition(double targetAngle){
    
    public double GetPos() {return GetRawPos() / TICKS_TO_DEG;}
    public double GetRawPos() {return motor.getCurrentPosition();}
    public double GetTargetPos() {return targetPosition / TICKS_TO_DEG;}
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
        atHome = !homeLimitSwitch.getState();
        if (atHome) homed = true;
        if (isHoming && homed) {Stop(); ResetEncoder(); }

        if (isBusy) {
            if (isHoming && !homed) {
                    rawSet(homePower);
                    if (homeTime.seconds() > 3) StartHome();
            } else {
                controller.setPID(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D);
                rawSet(controller.calculate(motor.getCurrentPosition(), targetPosition));
            }
        }


        if (InError() && AUTOSTOP) isBusy = false;
    }
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
}
