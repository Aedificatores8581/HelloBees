package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
    private static final double DEFAULT_WRIST_ANGLE = 75;
    private static final double Z_OFFSET = 12;

    //armToOrientation variables
    private Orientation target_angle;
    private Position target_position;
    private Orientation current_angle;
    private Position arm_position;
    private Orientation robot_orientation;
    private boolean arm_automation = false;
    private int arm_state = 0;
    private double arm_x = 0;
    private double arm_y = 0;
    private double arm_z = 0;
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
        arm_position = new Position(DistanceUnit.INCH,0,0,0,System.nanoTime());
        target_position = new Position(DistanceUnit.INCH,0,0,0,System.nanoTime());
        robot_orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0,0,0,System.nanoTime());
    }
    public void update() {
        pump.update();
        shoulder.Update();
        turret.Update();
        wrist.Update();
        fan.Update();
        fogger.Update();
        extension.Update();
        updatearm();
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

    //update arm current position
    private void updatearm(){
        arm_position.z = shoulder.GetHeight() + wrist.GetHeight();
        arm_position.y = turret.getPosition().y;
        arm_position.x =-( extension.GetPos() + shoulder.getExtension());
    }
    public void startarmToPosition(Position target){
        arm_automation = true;
        init_armToPosition(target);

    }
    public void stopArmToPosition(){
        arm_automation = false;
        arm_state = 0;
        wrist.Stop();
        turret.Stop();
        shoulder.Stop();
        turret.Stop();
        extension.Stop();
    }
    private void init_armToPosition(Position target){
        arm_state = 1;
        arm_automation = true;
        target_position = target;
        wrist.GoToAngle(DEFAULT_WRIST_ANGLE);
        turret.GoTo(.241);
    }
    private void armToPosition(){
        if (arm_state == 1){
            if(!turret.IsBusy()){
                arm_state = 2;
                extension.GoTo(Math.abs(target_position.x) - Math.abs(arm_x));
            }
        }
        if(arm_state == 2 && !extension.IsBusy()){
            arm_state = 3;
            shoulder.GoToHeight(target_position.z);
        }
        if(arm_state == 3 && !shoulder.IsBusy()){
            arm_automation = false;
            arm_state = 0;
        }
    }
    public boolean isArm_automation(){return arm_automation;}
    public int armAutoState(){return arm_state;}
    public Position getCurrent_Arm_Position(){
        return arm_position;
    }
}
