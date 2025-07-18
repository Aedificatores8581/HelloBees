package org.firstinspires.ftc.teamcode.susbsystems.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.susbsystems.LinkageExtension;

@TeleOp (name = "Linkage Extension Test", group = "SubsysTest")
public class linkageTest extends OpMode {
    LinkageExtension linkage;

    @Override
    public void init() {
        linkage = new LinkageExtension(hardwareMap);
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
