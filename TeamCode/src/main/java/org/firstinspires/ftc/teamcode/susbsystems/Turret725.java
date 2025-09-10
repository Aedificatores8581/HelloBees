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

//TODO 7/14/25 - Frank
// - confirm default 0 degree position and rotation direction
//    - affects GoTo()
// - detect if a given point is too close to the robot
// - integrate with extension system for a getTargetVector() function
// - latestDirection vector is a clunky implementation. I should streamline it
//    - current solution: add a static method to Vector2 to create a new polar vector
// - move getVectorTarget() to the full system class, as it needs arm and extension information


public class Turret725 {

    //Turret Constants
    //pid for movement
    public static final double TURRET_PID_P_DEFAULT = 0.001;
    public static final double TURRET_PID_I_DEFAULT = 0;
    public static final double TURRET_PID_D_DEFAULT = 0.00003;
    private double pCoef = TURRET_PID_P_DEFAULT, iCoef = TURRET_PID_I_DEFAULT, dCoef = TURRET_PID_D_DEFAULT;
    //2nd pid values for movement if needed
    public static final double S_PID_P_DEFAULT = 0.0015;
    public static final double S_PID_I_DEFAULT = 0;
    public static final double S_PID_D_DEFAULT = 0.000045;
    // turret min/max  analog potentiometer values
    private static final double TURRET_MIN_POSITION = .185;
    private static final double TURRET_MAX_POSITION = .315;

    private static final double TURRET_ERROR = .001;
    //changed this variable from TICKS_TO_DEG to DEG_TO_TICKS for accuracy
    private final double TURRET_POT_TO_DEG = 0.000483271;

    private DcMotorEx motor;

    private AnalogInput turretPot;
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    private double homePower = 0.2;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition = TURRET_MAX_POSITION;
    private double currentPosition;
    private double currentPower = 0;
    private double currentDegree;
    //represents the direction the turret has last been set to
    public Vector2 latestDirection = new Vector2();

    private PIDController controller;

    public boolean AUTOSTOP = true;

    public Turret725(HardwareMap hm){
        init(hm);
        setPID(pCoef, iCoef, dCoef);
    }
    private void init(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "turret");
        turretPot = hm.get(AnalogInput.class, "pot1");
        controller = new PIDController(pCoef, iCoef, dCoef);
    }
    public void setPID(double P, double I, double D){
        pCoef = P; iCoef = I; dCoef = D;
    }
    
    public void StartHome() {
        homePower = -homePower;
        isBusy = true;
        isHoming = true;
        homeTime.reset();
    }
    public void GoTo(Vector3 targetVector) {
        GoTo(GetTargetAngleDeg(targetVector));
    }

    public void GoTo(double targetAngle) {
        targetPosition = targetAngle * TURRET_POT_TO_DEG;
        //safety lock out
        if (targetPosition <= TURRET_MIN_POSITION) {
            targetPosition = TURRET_MIN_POSITION;
        }
        if (currentPosition >= TURRET_MAX_POSITION) {
            targetPosition = TURRET_MAX_POSITION;
        }
        isBusy = true;
    }
    public Vector2 getUnitVectorTarget(Vector3 targetVector){
        latestDirection.setFromPolar(1, GetTargetAngleRad(targetVector));
        return latestDirection;
    }
    //This function should go in the full system class
    //public Vector3 getVectorTarget(Vector3 targetVector, double extensionLength){return getUnitVectorTarget(targetVector).toVector3(0).add(0,0,extensionHeight).scalar(extensionLength + retractedLength);}

    public double GetTargetAngleRad(Vector3 targetVector){
        return Math.atan2(targetVector.y,targetVector.x);
    }
    public double GetTargetAngleDeg(Vector3 targetVector){ return Math.toDegrees(GetTargetAngleRad(targetVector));}
    public void GetTargetPosition(double targetAngle){/*Put something here*/}

    public double GetPos() {return (currentPosition / TURRET_POT_TO_DEG);}
    public double GetRawPos() {return turretPot.getVoltage();}
    public double GetTargetPos() {return (targetPosition / TURRET_POT_TO_DEG);}
    public double GetRawTargetPos() {return targetPosition;}
    public boolean InError() { return Math.abs(currentPosition - GetRawTargetPos()) < TURRET_ERROR;}
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
        isBusy = false;
        isHoming = false;
    }
    public void Update() {
        currentPosition = GetRawPos();

        if (currentPosition >= TURRET_MAX_POSITION) homed = true;
        if (isHoming && homed) {Stop();}

        if(isBusy && ((Math.abs(currentPosition - GetRawTargetPos()) < TURRET_ERROR))){Stop();};

        //if (InError()) {
           // if(AUTOSTOP) isBusy = false;
           // SetPIDCoef(S_PID_P_DEFAULT,S_PID_I_DEFAULT,S_PID_D_DEFAULT);
       // }
        //else { SetPIDCoef(TURRET_PID_P_DEFAULT,TURRET_PID_I_DEFAULT,TURRET_PID_D_DEFAULT); }

        if (isBusy) {
            if (isHoming && !homed) {
                    currentPower = homePower;
                    //if (homeTime.seconds() > 3) StartHome();
            } else {
                controller.setPID(pCoef, iCoef, dCoef);
                currentPower = controller.calculate(GetRawPos(), targetPosition);
            }
        }
        //safety lockouts
        if (currentPosition <= TURRET_MIN_POSITION) {
            currentPower = Math.min(Math.max(currentPower, -1), 0);
        }
        if (currentPosition >= TURRET_MAX_POSITION) {
            currentPower = Math.min(Math.max(currentPower, 0), 1);
        }
        rawSet(currentPower);
    }
    public void SetPIDCoef(double p,double i,double d) {pCoef = p; iCoef = i; dCoef = d;}
    public double GetPower() {return motor.getPower();}
    public boolean Homed() {return homed;}
}
