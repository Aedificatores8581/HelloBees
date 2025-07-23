package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.LinkageExtension725;

@TeleOp (name = "Linkage Extension725 Test", group = "SubsysTest")
public class linkageTest extends OpMode {
    LinkageExtension725 linkage;

    @Override
    public void init() {
        linkage = new LinkageExtension725(hardwareMap);
        linkage.StartHome();
    }

    @Override
    public void init_loop() {
        if (!linkage.Homed())
            linkage.Update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        linkage.Update();
    }

    @Override
    public void stop() {

    }
}
