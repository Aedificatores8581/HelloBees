package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.FullFoggerSubsystem;
import org.firstinspires.ftc.teamcode.util.ButtonBlock;

@TeleOp (name="Full Fogger Test")
public class fullFoggerTest extends OpMode {
    ButtonBlock startFogCycle, startFullTreatment;
    FullFoggerSubsystem foggerSystem;

    @Override
    public void init() {
        foggerSystem = new FullFoggerSubsystem(hardwareMap);

        startFogCycle = new ButtonBlock()
                .onTrue(() -> foggerSystem.startFogCycle());
        startFullTreatment = new ButtonBlock()
                .onTrue(() -> foggerSystem.startFullTreatment());
    }
    @Override
    public void init_loop() {

    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        startFogCycle.update(gamepad1.a);
        startFullTreatment.update(gamepad1.b);
        foggerSystem.update();
        telemetry.addLine("Controls:");
        telemetry.addLine("<A> - Start Fog Cycle");
        telemetry.addLine("<B> - Start Full Treatment");
        telemetry.addLine();
        telemetry.addData("Fog Cycle State", foggerSystem.getFogCycleState());
        telemetry.addData("Treatment State", foggerSystem.getTreatmentState());
        telemetry.addData("Cycle Count", foggerSystem.getCycleCount());
        telemetry.addLine();
        telemetry.addData("Tank Uses", foggerSystem.getTankUses()); // Tank Uses increments for every fog cycle and at 5 the pump refills to tank and the count resets to zero
        telemetry.update();
    }
    @Override
    public void stop() {
        foggerSystem.ShutOffRelays();
    }
}
