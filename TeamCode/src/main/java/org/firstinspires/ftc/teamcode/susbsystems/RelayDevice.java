package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RelayDevice {
    private DigitalChannel device;
    private ElapsedTime actionRunTime = new ElapsedTime();
    private double runTimeGoal;
    private boolean isBusy = false;
    private boolean currentState = false;
    public RelayDevice(HardwareMap hm, String deviceName) {
        device = hm.get(DigitalChannel.class, deviceName);
        device.setMode(DigitalChannel.Mode.INPUT);
    }
    private void SetState(boolean state) {
        device.setState(state); // This exists in case false is actually what turns the device on so it can be flipped in one spot
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
}
