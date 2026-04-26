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

    public enum Subsystems {
        DRIVE, SHOULDER, TURRET, WRIST, PUMP, FAN, FOGGER, EXTENSION
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
    private static final double SHOULDER_Z_OFFSET = 7;
    private static final double TURRET_Y_OFFSET = 5;
    private static final double EXTENSION_X_OFFSET = 3;
    private static final double WRIST_Z_OFFSET = 6;
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
    private boolean arm_is_busy = false;
    private boolean arm_homing = false;
    private boolean arm_homed = false;
    private boolean arm_last_position_bad = false;
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
        arm_position = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
        target_position = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
        robot_orientation = new Orientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 0, 0, 0, System.nanoTime());
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
        if (cycling) {
            cycle();
        }
        if (arm_automation) {
            armToPosition();
        }
        if (!arm_ready) {
            armReady();
        }
        if (fullCycleAutomation) {
            fullCycle();
        }
        arm_is_busy = shoulder.IsBusy() || turret.IsBusy() || wrist.IsBusy() || extension.IsBusy();
    }

    public void robot_drive(double power, double steering) {
        robot_drive.Set(power, steering);
    }

    public void doSomething(Subsystems subsystem) {
        switch (subsystem) {
            case PUMP:
                pump.ToggleState();
                break;
            case FAN:
                fan.ToggleState();
            default:
                break;
        }
    }

    public void runForTime(int seconds) {
        runForTime = seconds;
    }

    public void runForTicks(int ticks) {
        runForTicks = ticks;
    }

    //code used for the automated fog cycle
    //************************************
    //*************************************
    public void startFogCycle() {
        cycling = true;
        init_cycle();
    }

    public void stopFogCycle() {
        cycling = false;
        pump.TurnOff();
        fan.TurnOff();
        cyclecount = 0;
    }

    private void init_cycle() {
        cycling = true;
        pumpRunTime.reset();
        fogRunTime.reset();
        cyclecount = 0;
        pump.TurnOn();
        fogger.TurnOn();
    }

    private void cycle() {
        if (pumpRunTime.seconds() > PUMP_TIME) {
            pump.TurnOff();
        }
        if (cyclecount == 0 && fogRunTime.seconds() > FOG_TIME) {
            fanRunTime.reset();
            fan.TurnOn();
            cyclecount++;
        }
        if (fanRunTime.seconds() > FAN_TIME) {
            fan.ToggleState();
            fanRunTime.reset();
        }
        if (cyclecount > 0) {
            if (fogRunTime.seconds() > FOG_TIME) {
                fogger.ToggleState();
                fogRunTime.reset();
                cyclecount++;
            }
        }
        if (cyclecount > 16) {
            pump.TurnOff();
            fan.TurnOff();
            cycling = false;
        }
    }

    public int getCyclecount() {
        return cyclecount;
    }

    public boolean isCycling() {
        return cycling;
    }

    //code used for the move arm to point in space
    //************************************
    //*************************************

    //update arm current position
    private void updatearm() {
        // x left right of robot, right is positive.  (arm is on the front of the robot, electronics are on the back)
        // y forward / backward from robot - forward is positive
        // z up is positive
        // y, x, z 0,0,0 is camera zip tied to robot frame
        arm_position.z = shoulder.GetHeight() + SHOULDER_Z_OFFSET; //7 is the height of shoulder rotation above the camera
        arm_position.y = turret.getPosition().y + TURRET_Y_OFFSET; //6 is how far the end of the arm is in front of the camera
        arm_position.x = -(extension.GetPos() + shoulder.getExtension() - turret.getPosition().x);
        if (arm_automation) {
            if (!shoulder.IsBusy() && !turret.IsBusy() && !extension.IsBusy() && arm_state == 0) {
                arm_automation = false;
            }
        }

        if(shoulder.GetPos() <3 && shoulder.GetPos() > -3 && turret.GetRawPos() >.310 && extension.isAtHome()){
            //arm_homing = false;
            arm_homed = true;
        }
        else arm_homed = false;
    }

    public void startarmToPosition(Position target) {
        if(!arm_is_busy){
            //arm_automation = true;
            arm_last_position_bad = init_armToPosition(target);
        }
    }

    public void armHome() {
        //arm_automation = true;
        init_armToHome();
    }

    private void armReady() {
        if (shoulder.Homed() && turret.Homed() && extension.isAtHome()) {
            arm_ready = true;
        }
    }

    public void stopArmToPosition() {
        arm_automation = false;
        arm_state = 0;
        wrist.Stop();
        turret.Stop();
        shoulder.Stop();
        turret.Stop();
        extension.Stop();
    }

    private void init_armToHome() {
        if(arm_ready && !arm_is_busy) {
            arm_automation = true;
            arm_homing = true;
            extension.GoTo(0);
            arm_state = 1;
        }
    }
/*
0 (5 y , -7 x)
90 (15 y , -20 x)
135 (13 y, -24x)
180 (4 y, -28x)
225 (-3y, -24x)
*/
    private boolean init_armToPosition(Position target) {
        boolean valid_position = false;
        if (arm_ready&&!arm_is_busy&& arm_homed) {
            arm_state = 1;
            arm_automation = true;
            if(target.x > -20) stopArmToPosition();
            if(target.x < -38) stopArmToPosition();
            if(target.y > 15) stopArmToPosition();
            if(target.y <-3) stopArmToPosition();
            if(target.z <-10) stopArmToPosition();
            if(target.z > 10) stopArmToPosition();
            if(arm_automation){
                if (target.y > 3 && target.y < 7) {
                    turret.GoTo(Constants.ONEEIGHTY_DEGREES);
                    valid_position = true;
                }
                else if (target.y > 13.5) {
                    turret.GoTo(Constants.NINETY_DEGREES);
                    valid_position = true;
                } else if (target.y > 7 && target.y < 13.5) {
                    turret.GoTo(Constants.ONETHIRTYFIVE_DEGREES);
                    valid_position = true;
                } else if (target.y < 3) {
                    turret.GoTo(Constants.TWOTWENTYFIVE_DEGREES);
                    valid_position = true;
                } else {stopArmToPosition();}
            }
        }
        return valid_position;
    }

    private void armToPosition() {
        if(!arm_is_busy&&!arm_homing) {
            if (arm_state == 1 &&!turret.IsBusy()) {
                arm_state = 2;
                extension.GoTo(2);
                wrist.SetPos(.55);
            }
            if (arm_state == 2 && !extension.IsBusy()) {
                arm_state = 3;
                shoulder.GoToHeight(target_position.z - SHOULDER_Z_OFFSET);
            }
            if (arm_state == 3 && !shoulder.IsBusy()) {
                arm_state = 4;
                extension.GoTo(extension.GetPos() + Math.abs(Math.abs(arm_position.x) - Math.abs(target_position.x)) - EXTENSION_X_OFFSET);
            }
            if (arm_state == 4 && !extension.IsBusy()) {
                arm_state = 5;
                extension.GoTo(extension.GetPos() + 1);
                //wrist.GoToHeight(arm_position.z + WRIST_Z_OFFSET);
            }
            if (arm_state == 5 && !extension.IsBusy()) {
                arm_state = 0;
                //wrist.GoToHeight(6.75);
                arm_automation = false;
            }
        }
        else if(!arm_is_busy && arm_homing){
            if (arm_state == 1 &&!extension.IsBusy()) {
                arm_state = 2;
                shoulder.GoToAngle(0);
            }
            if (arm_state == 2 && !shoulder.IsBusy()) {
                arm_state = 3;
                turret.GoTo(Constants.TURRET_MAX_POSITION);
            }
            if (arm_state == 3 && !turret.IsBusy()) {
                arm_state = 0;
                arm_automation = false;
                arm_homing = false;
            }
        }
    }

    public boolean isArm_automation() {return arm_automation;}
    public boolean isArm_ready() {return arm_ready;}
    public int armAutoState() {return arm_state;}
    public Position getCurrent_Arm_Position() {return arm_position;}
    public boolean isArmBusy(){return arm_is_busy;}
    public boolean isArmHomed(){return arm_homed;}
    public boolean isArm_last_position_bad(){return arm_last_position_bad;}

    //shoulder functions
    //**************************************************************************************
    //**************************************************************************************
    public void shoulderHome() {shoulder.Home();}

    public void shoulderSetPower(double power) {shoulder.SetPower(power);}

    public void shoulderStop() {shoulder.Stop();}

    public boolean shoulderIsBusy() {
        return shoulder.IsBusy();
    }
    //turret functions
    //**************************************************************************************
    //**************************************************************************************
    public boolean turretIsBusy(){return turret.IsBusy();}
    public double turretPosition(){return turret.GetRawPos();}
    public double turretTargetPosition(){return turret.GetTargetPosition();}
    public void moveTurretManual(double power){
        if (!turret.IsBusy()) turret.SetPower(power);
        else if (Math.abs(power) > 0) {
            turret.Stop();
            turret.SetPower(power);
        }
    }
    public void moveTurret(int degrees){
        if(!arm_is_busy) {
            if (degrees == 0) {
                turret.GoTo(Constants.ZERO_DEGREES);
            }
            if (degrees == 45) {
                turret.GoTo(Constants.FORTYFIVE_DEGREES);
            }
            if (degrees == 90) {
                turret.GoTo(Constants.NINETY_DEGREES);
            }
            if (degrees == 135) {
                turret.GoTo(Constants.ONETHIRTYFIVE_DEGREES);
            }
            if (degrees == 180) {
                turret.GoTo(Constants.ONEEIGHTY_DEGREES);
            }
            if (degrees == 225) {
                turret.GoTo(Constants.TWOTWENTYFIVE_DEGREES);
            }
        }
    }
    //extension functions
    //**************************************************************************************
    //**************************************************************************************
    public double getExtensionTarget() {return extension.GetTargetPos();}

    public boolean isBusyExtension() {return extension.IsBusy();}
    public void moveExtensionManual(double power) {
        if (!extension.IsBusy()) extension.SetPower(power);
        else if (Math.abs(power) > 0) {
            extension.Stop();
            extension.SetPower(power);
        }
    }
    public boolean isExtensionHome(){return extension.isAtHome();}
    //wrist functions
    //**************************************************************************************
    //**************************************************************************************
    public double getWristHeight() {return wrist.GetHeight();}
    public double getWristLength() {return wrist.GetLength();}

    public void moveWristManual(double power) {
        if(!wrist.IsBusy()){
            if (power > 0) {wrist.SetPos(wrist.GetRawPos() + .001);}
            if (power < 0) {wrist.SetPos(wrist.GetRawPos() - .001);}
        }
    }

    //armToPosition and Spray
    //**************************************************************************
    //**************************************************************************


    public void startFullCycle(Position target){
        if(arm_ready){
            fullCycleAutomation = true;
            fullCycleState = 1;
            arm_last_position_bad = init_armToPosition(target);
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
        if(fullCycleState == 3 && !arm_automation){
            fullCycleState = 4;
            target_position.y = 7;
            arm_last_position_bad = init_armToPosition(target_position);
        }
        if(fullCycleState == 4 && !arm_automation){
            fullCycleState++;
            init_cycle();
        }
        if(fullCycleState == 5 && !cycling){
            fullCycleState++;
            init_armToHome();
        }
        if(fullCycleState ==6 && !arm_automation){
            fullCycleState = 0;
            fullCycleAutomation = false;
        }
    }
    public int getFullCycleState(){return fullCycleState;}
    public boolean isFullCycleAutomation(){return fullCycleAutomation;}
}
