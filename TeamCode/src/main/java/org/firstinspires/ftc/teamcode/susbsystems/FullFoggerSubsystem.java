package org.firstinspires.ftc.teamcode.susbsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FullFoggerSubsystem {
    public int pumpAmount = 15000;
    public int targetCycleCount = 3, tankUsesBeforeRefill = 5;
    public double fanTime = 3, fogTime = 3;
    private int cycleCount = 0;
    private int tankUses = 0; // placeholder variable in case a tank is full sensor is added
    private int fogCycleState = 0;
    private int treatmentState = 0;
    public ElapsedTime fanTimer, fogTimer;

    public RelayDevice fan, fogger;
    public Pump_Subsystem pump;

    public FullFoggerSubsystem(HardwareMap hw) {
        fan = new RelayDevice(hw, "valve1");
        fogger = new RelayDevice(hw, "compressor1");
        pump = new Pump_Subsystem(hw, "pump");

        fanTimer = new ElapsedTime();
        fogTimer = new ElapsedTime();

        fan.TurnOff();
        fogger.TurnOff();
    }
    public void update() {
        if (fanTimer.seconds() > fanTime && fan.GetState())
            fan.TurnOff();
        if (fogTimer.seconds() > fogTime && fogger.GetState())
            fogger.TurnOff();

        updateFogCycle();
        updateTreatment();

        if (tankUses >= tankUsesBeforeRefill) { fillTank();}

        pump.update();
    }
    public void fillTank() {
        pump.RunForTicks(pumpAmount);
        tankUses = 0;
    }
    public void startFullTreatment() {
        treatmentState = 1;
        cycleCount = 0;
    }
    public void updateTreatment() {
        switch (treatmentState) {
            case 0: break;
            case 1: startFogCycle(); cycleCount++;treatmentState++;break;
            case 2:
                if (fogCycleState == 0) {
                    if (cycleCount >= targetCycleCount) {
                        treatmentState = 0;
                    } else {
                        treatmentState = 1;
                    }
                }
                break;
        }
    }
    public void startFogCycle() {
        tankUses++;
        fogCycleState = 1;
    }
    public void updateFogCycle() {
        switch (fogCycleState) {
            case 0: break;
            case 1: fogger.TurnOn();fogTimer.reset();fogCycleState++;break;
            case 2: if (!fogger.GetState()) fogCycleState++;break;
            case 3: fan.TurnOn();fanTimer.reset();fogCycleState++;break;
            case 4: if (!fan.GetState()) fogCycleState = 0;break;
        }
    }

}
