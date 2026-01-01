package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Util;

public class Pump_Subsystem { // Working but Could be Missing Some Needed Functions
   //Pump Constants
    private static final double PUMP_POWER = 1;
    private DcMotorEx motor;
    private ElapsedTime actionRunTime = new ElapsedTime();
    private double runTimeGoal;
    private boolean isBusy = false;
    private boolean currentState = false;
    private int currentPosition;
    private double currentPower = 0;
    private int targetTicks = 0;
    private boolean usingRunToPosition = false;
    private boolean usingRunToTime = false;
    private boolean locked;
    private boolean direction = true; //forward is positive and back = false
    public Pump_Subsystem(HardwareMap hm, String deviceName) {
        motor = hm.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setTargetPosition(targetTicks);
    }
    public void TurnOn() {
        if(direction) currentPower = PUMP_POWER; else currentPower = -PUMP_POWER;
        rawSet(currentPower);
        isBusy = true;
    }
    public void TurnOff() {
        currentState = false;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        usingRunToPosition = false;
        usingRunToTime = false;
        currentPower = 0;
        rawSet(currentPower);
        isBusy = false;
    }
    public void ResetEncoder() {
        currentPower = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentPosition = motor.getCurrentPosition();
    }
    private void rawSet(double power) {
        motor.setPower(Util.IntClamp(power));
    }
    public void SetPower(double power) {
        currentPower = power;
    }
    public void RunForSeconds(double seconds) {
        runTimeGoal = seconds;
        usingRunToTime = true;
        actionRunTime.reset();
        TurnOn();
    }
    public void RunForTicks(int ticks) {
        ResetEncoder();
        targetTicks = ticks;
        usingRunToPosition = true;
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TurnOn();
    }
    public void ToggleState() {
        if (!isBusy){TurnOn();}
        else TurnOff();
    }
    public void ToggleDirection() {
        direction = !direction;
    }
    public void update() {
        currentState = currentPower != 0;
        currentPosition = motor.getCurrentPosition();
        rawSet(currentPower);
        if(isBusy){
            if (usingRunToPosition && !motor.isBusy()){TurnOff();}
            if(usingRunToTime && actionRunTime.seconds() > runTimeGoal){TurnOff();}
        }

    }
    public boolean GetState() {return currentState;}
    public int GetPos() {return currentPosition;}
    public double GetPower() {return currentPower;}
    public int GetTarget() {return targetTicks;}
    public boolean isBusy() {return isBusy;}
    public boolean Direction() {return direction;}
}
