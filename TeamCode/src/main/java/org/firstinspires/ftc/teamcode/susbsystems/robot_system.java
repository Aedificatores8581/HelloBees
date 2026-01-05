package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;

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
    private static final double SHOULDER_Z_OFFSET = 3;
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
    private boolean arm_ready = false;
    //buttons

    //local variables
    int runForTime = 0; //seconds
    int runForTicks = 0; //encoder ticks

    //full cycle
    //*************************************************************************************************************************
    //*************************************************************************************************************************
    private boolean fullCycleAutomation = false;
    private int fullCycleState = 0;

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
        turret.StartHome();
        arm_position = new Position(DistanceUnit.INCH,0,0,0,System.nanoTime());
        target_position = new Position(DistanceUnit.INCH,0,0,0,System.nanoTime());
        robot_orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,0,0,0,System.nanoTime());
        arm_ready = false;
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
        if(!arm_ready){armReady();}
        if(fullCycleAutomation){fullCycle();}
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
    private void updatearm() {
        // x left right of robot, right is positive.  x = 0 is top of center turrent rotation
        // y forward / backward from robot - forward is positive y = 0 is top of center turret rotation point
        // z up is positive z = 0 is top of center turret rotation point
        arm_position.z = shoulder.GetHeight() + SHOULDER_Z_OFFSET; //3 is the height of shoulder rotation above the center of turret rotation
        arm_position.y = turret.getPosition().y;
        arm_position.x = -(extension.GetPos() + shoulder.getExtension()- turret.getPosition().x);
        if (arm_automation) {
            if (!shoulder.IsBusy()&&!turret.IsBusy()&&!extension.IsBusy()&& arm_state ==0) {
                arm_automation = false;
            }
        }
    }
    public void startarmToPosition(Position target){
        arm_automation = true;
        init_armToPosition(target);
    }
    public void armHome()
    {
        arm_automation = true;
        init_armToHome();
    }
    private void armReady(){
        if(shoulder.Homed()&&turret.Homed()&& extension.isAtHome()){
            arm_ready = true;
        }
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
    private void init_armToHome(){
        arm_automation = true;
        shoulder.GoToAngle(0);
        extension.GoTo(0);
        turret.GoTo(Constants.TURRET_MAX_POSITION);
    }
    private void init_armToPosition(Position target){
        if(arm_ready) {
            arm_state = 1;
            arm_automation = true;
            target_position = target;
            wrist.GoToAngle(DEFAULT_WRIST_ANGLE);
            if(target.y>=3 && target.y<=8){
                turret.GoTo(Constants.ONETHIRTYFIVE_DEGREES);}
            else if(target.y>-3 && target.y<3){
                turret.GoTo(Constants.ONEEIGHTY_DEGREES);}
            else if(target.y>8 && target.y<11){
                turret.GoTo(Constants.NINETY_DEGREES);}
            else if(target.y<=-3 && target.y>-8){
                turret.GoTo(Constants.TWOTWENTYFIVE_DEGREES);}
            else{stopArmToPosition();}
        }
    }
    private void armToPosition(){
        if (arm_state == 1){
            if(!turret.IsBusy()){
                arm_state = 2;
                extension.GoTo(4);
            }
        }
        if(arm_state == 2 && !extension.IsBusy()){
            arm_state = 3;
            shoulder.GoToHeight(target_position.z-SHOULDER_Z_OFFSET);
        }
        if(arm_state == 3 && !shoulder.IsBusy()){
            arm_state = 4;
            extension.GoTo(6);
        }
        if(arm_state == 4 && !extension.IsBusy()){
            arm_state = 5;
            extension.GoTo(7);
            wrist.GoToHeight(6.75);
        }
        if(arm_state == 5 && !extension.IsBusy()){
            arm_state = 0;
            wrist.GoToHeight(6.75);
            arm_automation = false;
        }
    }
    public boolean isArm_automation(){return arm_automation;}
    public boolean isArm_ready(){return arm_ready;}
    public int armAutoState(){return arm_state;}
    public Position getCurrent_Arm_Position(){
        return arm_position;
    }

    //shoulder functions
    //**************************************************************************************
    //**************************************************************************************
    public void shoulderHome(){shoulder.Home();}
    public void shoulderSetPower(double power){shoulder.SetPower(power);}
    public void shoulderStop(){shoulder.Stop();}
    public boolean shoulderIsBusy(){return shoulder.IsBusy();}


    //armToPosition and Spray
    //**************************************************************************
    //**************************************************************************
    public void startFullCycle(Position target){
        if(arm_ready){
            fullCycleAutomation = true;
            fullCycleState = 1;
            init_armToPosition(target);
        }
    }

    public void stopFullCycle(){
        fullCycleAutomation = false;
        fullCycleState = 0;
        stopArmToPosition();
        stopFogCycle();
    }
    private void fullCycle(){
        if(fullCycleState == 1 && !arm_automation){
            fullCycleState++;
            init_cycle();
        }
        if(fullCycleState == 2 && !cycling){
            fullCycleState++;
            init_armToHome();
        }
        if(fullCycleState == 3 && arm_ready){
            fullCycleState = 0;
            fullCycleAutomation = false;
        }
    }
    public int getFullCycleState(){return fullCycleState;}
    public boolean isFullCycleAutomation(){return fullCycleAutomation;}
}
