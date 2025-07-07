package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RelayDevice { // Working but Could be Missing Some Needed Functions
    private DigitalChannel device;
    private ElapsedTime actionRunTime = new ElapsedTime();
    private double runTimeGoal;
    private boolean isBusy = false;
    private boolean currentState = false;
    private boolean locked;
    public RelayDevice(HardwareMap hm, String deviceName) {
        device = hm.get(DigitalChannel.class, deviceName);
        InputOn();
    }
    private void SetState(boolean state) {
        device.setState(!state); // This exists in case false is actually what turns the device on so it can be flipped in one spot
    }
    public void TurnOn() {
        currentState = true;
        SetState(currentState);
    }
    public void TurnOff() {
        currentState = false;
        SetState(currentState);
        isBusy = false;
    }
    public void ToggleState() {
        currentState = !currentState;
        SetState(currentState);
    }
    public void RunForSeconds(double seconds) {
        runTimeGoal = seconds;
        actionRunTime.reset();
        isBusy = true;
        TurnOn();
    }
    public void Update() {
        if (isBusy && actionRunTime.seconds() > runTimeGoal) {
            TurnOff();
        }
    }
    public boolean GetState() {
        return currentState;
    }
    public void FullShutOff() {TurnOff();InputOff();}
    public void InputOff() { device.setMode(DigitalChannel.Mode.INPUT); locked = true; }
    public void InputOn() { device.setMode(DigitalChannel.Mode.OUTPUT); locked = false; SetState(currentState); }
    public void ToggleInput() {
        if (locked)
            InputOn();
        else
            InputOff();
    }
    public boolean InputState() {return locked;}
}
