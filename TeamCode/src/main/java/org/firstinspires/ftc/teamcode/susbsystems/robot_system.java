package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class robot_system {
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
}
