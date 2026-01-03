package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class robot_system {
    // robot constants

    //robot subsystems
    Drive robot_drive;
    RelayDevice fan;
    RelayDevice fogger;
    Pump_Subsystem pump;
    Arm725 shoulder;
    LinkageExtension725 extension;
    Turret725 turret;
    Wrist wrist;
    public enum Subsystems{
        DRIVE,SHOULDER,TURRET,WRIST,PUMP,FAN,FOGGER,EXTENSION
    }


    //fog cycle
    //****************************************************************************************************************************
    //****************************************************************************************************************************
    //fog Cycle Constants
    private static final double PUMP_TIME = 5;
    private static final double FOG_TIME = 2;
    private static final double FAN_TIME = .5;

    //fog cycle variables
    //********************
    private ElapsedTime fogRunTime = new ElapsedTime();
    private ElapsedTime fanRunTime = new ElapsedTime();
    private ElapsedTime pumpRunTime = new ElapsedTime();
    boolean cycling = false;
    int cyclecount = 0;

    //armToOrientation
    //*************************************************************************************************************************
    //*************************************************************************************************************************
    //armToOrientation Constants


    //armToOrientation variables
    private Orientation target_position;
    private Orientation current_position;
    private Orientation robot_position;
    private boolean arm_automation = false;

    //buttons

    //local variables
    int runForTime = 0; //seconds
    int runForTicks = 0; //encoder ticks

    public robot_system(HardwareMap hm) {
        pump = new Pump_Subsystem(hm, "pump");
        shoulder = new Arm725(hm);
        turret = new Turret725(hm);
        wrist = new Wrist(hm);
        fan = new RelayDevice(hm, "valve1");
        fogger = new RelayDevice(hm, "compressor1");
        extension = new LinkageExtension725(hm);
        robot_drive = new Drive(hm);
        init();
    }
    private void init() {
        extension.StartHome();
    }
    public void update() {
        pump.update();
        shoulder.Update();
        turret.Update();
        wrist.Update();
        fan.Update();
        fogger.Update();
        extension.Update();
        if(cycling){cycle();}
        if(arm_automation){armToPosition();}
        }
        public void robot_drive(double power, double steering) {
            robot_drive.Set(power, steering);
        }
        public void doSomething(Subsystems subsystem){
        switch(subsystem){
            case PUMP:
                pump.ToggleState();
                break;
            case FAN:
                fan.ToggleState();
            default:
                break;
        }
        }
        public void runForTime(int seconds){
            runForTime = seconds;
        }
    public void runForTicks(int ticks){
            runForTicks = ticks;
        }

    //code used for the automated fog cycle
    //************************************
    //*************************************
    public void startFogCycle(){
        cycling = true;
        init_cycle();
    }
    public void stopFogCycle(){
        cycling = false;
        pump.TurnOff();
        fan.TurnOff();
        cyclecount = 0;
    }
    private void init_cycle(){
        cycling = true;
        pumpRunTime.reset();
        fogRunTime.reset();
        cyclecount = 0;
        pump.TurnOn();
        fogger.TurnOn();
    }
    private void cycle(){
        if (pumpRunTime.seconds()>PUMP_TIME){pump.TurnOff();}
        if (cyclecount ==0 && fogRunTime.seconds() >FOG_TIME){
            fanRunTime.reset();
            fan.TurnOn();
            cyclecount++;
        }
        if (fanRunTime.seconds()> FAN_TIME){
            fan.ToggleState();
            fanRunTime.reset();
        }
        if (cyclecount > 0){
            if(fogRunTime.seconds()>FOG_TIME){
                fogger.ToggleState();
                fogRunTime.reset();
                cyclecount++;
            }
        }
        if (cyclecount >16){
            pump.TurnOff();
            fan.TurnOff();
            cycling = false;
        }
    }
    public int getCyclecount(){return cyclecount;}
    public boolean isCycling(){return cycling;}

    //code used for the move arm to point in space
    //************************************
    //*************************************
    public void startarmToPosition(Orientation target){
        arm_automation = true;
        init_armToPosition(target);

    }
    public void stopArmToPosition(){
        arm_automation = false;

    }
    private void init_armToPosition(Orientation target){
        arm_automation = true;
        target_position = target;
        robot_position.firstAngle = 0;
        robot_position.secondAngle = 0;
        robot_position.thirdAngle = 0;
    }
    private void armToPosition(){
        //do stuff
    }
    public boolean isArm_automation(){return cycling;}
}
