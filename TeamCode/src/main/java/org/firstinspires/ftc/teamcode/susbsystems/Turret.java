package org.firstinspires.ftc.teamcode.susbsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.Util;

public class Turret { // STILL WIP
    private final double TICKS_TO_DEG = 5900d/90d;

    private DcMotorEx motor;
    private DigitalChannel homeLimitSwitch;
    private boolean isBusy = false;
    private boolean atHome, homed = false, isHoming = false;
    private double homePower = 0.3;
    private ElapsedTime homeTime = new ElapsedTime();
    private double targetPosition;

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
    public void GoTo(double targetPosition) {
        this.targetPosition = targetPosition*TICKS_TO_DEG;
        isBusy = true;
    }
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

        if (isBusy) { // TODO: Fix Excessive Nesting
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
