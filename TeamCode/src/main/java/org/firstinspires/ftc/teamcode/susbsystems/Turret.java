package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private final double TICKS_TO_DEG = 6000/90;
    private final double ERROR_RANGE = 100;

    private DcMotorEx motor;
    private DigitalChannel homeLimitSwitch;
    private boolean atHome;
    private boolean isBusy = false;
    private boolean homed = false;

    private double targetPosition;


    public Turret(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "turret");
    }
    public double GetPos() {return motor.getCurrentPosition() * TICKS_TO_DEG;}
    public double GetTargetPos() {return targetPosition;}
    public boolean InError() { return Math.abs(GetPos() - GetTargetPos()) < ERROR_RANGE/2;}
    public boolean IsBusy() {return isBusy;}
    public void Update() {
        atHome = !homeLimitSwitch.getState();



        if (InError()) isBusy = false;
    }
    //TODO: Implement Turret PID, Movement Functions, and Homing Functions
}
