/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.matrices.*;


import java.util.List;
import java.util.LinkedList;
import java.util.Queue;

@Config
@TeleOp(name="Hello Bees-Java", group="Iterative OpMode")
public class Hello_Bees_Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo rightdrive;
    private CRServo leftdrive;
    private Servo wrist;
    private DigitalChannel valve1;
    private DcMotor linkage;
    private DigitalChannel compressor1;
    private DcMotor pump;
    private DigitalChannel turret_home;
    private DcMotorEx turret;
    private DigitalChannel front_limit;
    private DigitalChannel rear_limit;
    private CRServo shoulder;
    private AnalogInput pot1;
    
    boolean AutoBlock;
    boolean PumpButtonBlock;
    boolean ButtonBlockValuePump;
    boolean CompressorButtonBlock;
    boolean ValveButtonBlock;
    boolean ButtonBblock;
    boolean ButtonAblock;
    boolean ButtonA1block = false;
    boolean ButtonB1block = false;
    boolean ButtonX1block = false;
    boolean ButtonY1block = false;
    double armPos = 0;
    double shoulder_angle = 0;
    double linkageMotorPower = 0;
    double wristServoPosition = 0;
    double shoulderMotorPower = 0;
    double turretMotorPower = 0;
    boolean armAutomation = false;
    double pumpMotorPower = 0;
    boolean fanRelay;
    boolean foggerRelay;
    boolean frontLinkageLimit;
    boolean rearLinkageLimit;
    boolean turretHomeSensor;
    double rightdriveMotorPower = 0;
    double leftdriveMotorPower = 0;
    double linkageMotorPosition;
    double turretMotorPosition;
    boolean turretHomed = false;
    int automationState = 0;
    boolean armStowed = false;
    public int turret_target = 0;
    public int turret_stowed = 6000;
    //automation targets
    public int turret_movement_target = 0;
    public double shoulder_target = 2;
    public double shoulder_movement_target = 0;
    int linkage_target = 0;
    double wrist_target = 0;
    public double shoulder_stowed = .5;
    private PIDController turret_controller;
    private PIDController shoulder_controller;
    public static double shoulder_p = 2, shoulder_i = 0, shoulder_d = .1;
    public static double p =0.001, i=0, d = 0.00003;
    
    private final double turret_ticks_in_degree = 64.47;
    private final double TURRET_ENCODER_TO_RADIANS = toRadians(turret_ticks_in_degree);
    private final double ARM_ENCODER_IN_DEGREE = .0122222;
    private final double ARM_ENCODER_TO_RADIANS = toRadians(ARM_ENCODER_IN_DEGREE);
    private final double QR_distance_away = 6; //desired distance away from QR code, inches
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Position cameraPosition = new Position(DistanceUnit.INCH,0, 0, 0, 0); // old values, currently innacurate
    Position cameraRelCoords;
    final double YAW_CAM = 0.785398, PITCH_CAM = 0, ROLL_CAM = 0;
    //extension measurements
    double EXTENSION_MOTOR_OFFSET = 4.5;
    double EXTENSION_ENCODER_TO_RADIANS = toRadians(537.7/360);
    final double MIN_EXTENSION_LENGTH = 10.25;
    final double EXTENSION_RANGE = 13.75;
    double EXTENSION_ENCODER_TO_INCHES = 225/ EXTENSION_RANGE;
    final double MAX_EXTENSION_LENGTH = MIN_EXTENSION_LENGTH + EXTENSION_RANGE;
    //arm or shoulder measurements
    final double PIVOT_HEIGHT = 3.5;
    //final double z_0 = Z_CAM - PIVOT_HEIGHT;
    final double ARM_LENGTH = 16.5;
    final double LINKAGE_LENGTH_1 = 12;
    final double LINKAGE_LENGTH_2 = 13;
    double LINK_2_ATTACHMENT_HEIGHT = 1;
    final double SLIDER_HEIGHT = 1.7;
    final double TIP_TO_PIVOT_DISTANCE = 8;
    final double RETRACTED_LINK_1_ANGLE = Math.atan2(SLIDER_HEIGHT, MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE)+Math.acos((Math.pow(LINKAGE_LENGTH_1,2)+Math.pow(SLIDER_HEIGHT,2)+Math.pow(MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE,2)-Math.pow(LINKAGE_LENGTH_2,2))/2/(MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE)/LINKAGE_LENGTH_1);
    Position rotation_matrix;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    List<AprilTagDetection> currentDetections;
    Position detectedPosition;
    YawPitchRollAngles detectedYPRA;
    int detectedTageID;
    Queue<Double> loopTimes = new LinkedList<>();
    int loop_Window_Size = 10;
    Double averageLoopTime = 0.0;
    //target distance below the AprilTag
    double AprilTag_distance_away = 3;
    boolean arm_to_AprilTag = false;
    AprilTagDetection currentAprilTag;
    Position robotRelCoords;
    double arm_angle;
    Position aprilTag_Target_Left = new Position(DistanceUnit.INCH,0, 0, 0, 0); //Tag 584 "Jonah"
    Position aprilTag_Target_Right = new Position(DistanceUnit.INCH,0, 0, 0, 0); //Tag 583 "Nemo"

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initAprilTag();
        rotation_matrix = new Position (DistanceUnit.INCH,-8, -7, 13, 0);
        //RETRACTED_LINK_1_ANGLE = asin(pow((MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE),2) + pow(SLIDER_HEIGHT,2)+ pow(LINKAGE_LENGTH_1,2)-pow(LINKAGE_LENGTH_2,2)) / (2* LINKAGE_LENGTH_1 * sqrt(pow((MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE),2)+ pow(SLIDER_HEIGHT,2))) - atan2((MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE) , SLIDER_HEIGHT);	//radians

        telemetry.addData("Status", "Initialized");
        rightdrive = hardwareMap.get(CRServo.class, "rightdrive");
        leftdrive = hardwareMap.get(CRServo.class, "leftdrive");
        wrist = hardwareMap.get(Servo.class, "wrist");
        valve1 = hardwareMap.get(DigitalChannel.class, "valve1");
        linkage = hardwareMap.get(DcMotor.class, "linkage");
        compressor1 = hardwareMap.get(DigitalChannel.class, "compressor1");
        pump = hardwareMap.get(DcMotor.class, "pump");
        turret_home = hardwareMap.get(DigitalChannel.class, "turret_home");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        front_limit = hardwareMap.get(DigitalChannel.class, "front_limit");
        rear_limit = hardwareMap.get(DigitalChannel.class, "rear_limit");
        shoulder = hardwareMap.get(CRServo.class, "shoulder");
        pot1 = hardwareMap.get(AnalogInput.class, "pot1");

        rightdrive.setDirection(CRServo.Direction.REVERSE);
        leftdrive.setDirection(CRServo.Direction.REVERSE);
        wrist.setPosition(0.5);
        valve1.setMode(DigitalChannel.Mode.INPUT);
        linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        compressor1.setMode(DigitalChannel.Mode.INPUT);
        ButtonBlockValuePump = false;
        CompressorButtonBlock = false;
        ValveButtonBlock = false;
        PumpButtonBlock = false;
        ButtonBblock = false;
        AutoBlock = false;
        ButtonAblock = false;
        turret_controller = new PIDController(p, i ,d);
        shoulder_controller = new PIDController(shoulder_p, shoulder_i, shoulder_d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        loopTimes.offer(runtime.milliseconds());
        loopTimes.offer(runtime.milliseconds());
        if(loopTimes.size() > loop_Window_Size){
            loopTimes.poll();
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        sensorRead();
        telemetry();
        home_turret();
    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        valve1.setMode(DigitalChannel.Mode.OUTPUT);
        valve1.setState(true);
        // Put run blocks here.
        compressor1.setMode(DigitalChannel.Mode.OUTPUT);
        compressor1.setState(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Put loop blocks here.
        sensorRead();
        telemetry ();
        controllersGetValues ();
        if(armAutomation){
            automaticSpray();
        }
        if(arm_to_AprilTag && !currentDetections.isEmpty()){
            currentAprilTag = currentDetections.get(0);
            cameraRelCoords = currentAprilTag.robotPose.getPosition();
            robotRelCoords = getRobotRelativeCoordinate(cameraRelCoords);
            //turret_target_AprilTag = ;
        }
        actuatorCommands();
        loopTimes.offer(runtime.milliseconds());
        if(loopTimes.size() > loop_Window_Size){
            loopTimes.poll();
        }
        double loop_time_totalDifference = 0;
        Double[] loopTimesArray = loopTimes.toArray(new Double [0]); // Convert queue to array

        for (int i = 0; i < loopTimesArray.length - 1; i++) {
            loop_time_totalDifference += loopTimesArray[i + 1] - loopTimesArray[i]; // Calculate difference between consecutive timestamps
        }
        averageLoopTime = loop_time_totalDifference / (loopTimesArray.length - 1);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        visionPortal.close();
    }

    private void telemetry (){
        if(arm_to_AprilTag) automationTelemetryTest();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addLine(String.format("Left %6.1f %6.1f %6.1f  (inch)", aprilTag_Target_Left.x,
                aprilTag_Target_Left.y, aprilTag_Target_Left.z));
        telemetry.addLine(String.format("Right %6.1f %6.1f %6.1f  (inch)", aprilTag_Target_Right.x,
                aprilTag_Target_Right.y, aprilTag_Target_Right.z));
        telemetry.addLine(String.format("Auto: State %d Enabled %b Stowed %b Homed %b",automationState, armAutomation, armStowed, turretHomed));
        telemetry.addLine(String.format("Link: Pos %d Front %b Rear %b Busy %b",(int) linkageMotorPosition,frontLinkageLimit, rearLinkageLimit, linkage.isBusy()));
        telemetry.addLine(String.format("Fogger(False on): Fan %b Fog %b Pump (%.2f)", foggerRelay, fanRelay, pumpMotorPower));
        telemetry.addLine(String.format("Arm Loc Turret: Pos %d Target %d Angle (%.1f)",(int)turretMotorPosition, turret_target, (turretMotorPosition/turret_ticks_in_degree)-93));
        telemetry.addLine(String.format("Arm Loc Wrist: (%.2f) Shoulder: Raw (%.2f), Angle (%.1f)", wristServoPosition, armPos, shoulder_angle));
        telemetry.addLine(String.format("Status: Run Time: %s Average Loop: %.0f", runtime.toString(), averageLoopTime));
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftdriveMotorPower, rightdriveMotorPower);

        //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }
    private void home_turret(){
        if(!turretHomed){
            turret.setPower(-.3);
            if(!turretHomeSensor){
                turret.setPower(0);
                turretHomed = true;
            }
        }
    }
    private void sensorRead () {
        //shoulder position
        armPos = pot1.getVoltage();
        //shoulder_angle = (270*armPos +445.5 - Math.sqrt((((270 * armPos) + 445.5) * ((270 * armPos) + 445.5)) + ((4 * armPos) * ((36450 * armPos) + 120135))))/(2*armPos);
        shoulder_angle = armPos * 81.8;
        foggerRelay = compressor1.getState();
        fanRelay = valve1.getState();
        frontLinkageLimit = front_limit.getState();
        rearLinkageLimit = rear_limit.getState();
        turretHomeSensor = turret_home.getState();
        turretMotorPosition = turret.getCurrentPosition();
        linkageMotorPosition = linkage.getCurrentPosition();
        currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                detectedTageID = detection.id;
                if(detection.id == 584) aprilTag_Target_Left = detection.robotPose.getPosition();
                if(detection.id == 583) aprilTag_Target_Right = detection.robotPose.getPosition();
                //detectedPosition = new Position(DistanceUnit.INCH, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y,detection.robotPose.getPosition().z, 0);
                //detectedYPRA = new YawPitchRollAngles(AngleUnit.DEGREES, detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES), detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES), 0);
            }
        }
    }

    private void actuatorCommands (){
        armStowed = !rearLinkageLimit && !((abs(turret_stowed - turretMotorPosition)) > 100) && !(armPos < shoulder_stowed - .05) && !(armPos > shoulder_stowed + .05);
        //drivetrain motors
        leftdriveMotorPower = Math.min(Math.max(leftdriveMotorPower, -.3),.3); //movement safety
        rightdriveMotorPower = Math.min(Math.max(rightdriveMotorPower, -.3),.3); //movement safety
        leftdrive.setPower(leftdriveMotorPower);
        rightdrive.setPower(rightdriveMotorPower);

        //wrist servo
        wristServoPosition = Math.min(Math.max(wristServoPosition, .44),1); //movement safety
        wrist.setPosition(wristServoPosition);

        //linkage movement
        if (!frontLinkageLimit) {
            linkageMotorPower = Math.min(Math.max(linkageMotorPower, 0),1); //movement safety
        }
        if (!rearLinkageLimit) {
            if(!armAutomation){
            linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);}
            linkageMotorPower = Math.min(Math.max(linkageMotorPower, -1),0); //movement safety
        }
        linkage.setPower(linkageMotorPower);

        //turret movement
        if(armAutomation) {
            turret_controller.setPID(p, i, d);
            turretMotorPower = turret_controller.calculate(turretMotorPosition, turret_target);
        }
        if (!turret_home.getState()) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretMotorPower = Math.min(Math.max(turretMotorPower, -.6),.6); //movement safety
        if(turretHomed && turretMotorPosition <-2000){
            turretMotorPower = Math.min(Math.max(turretMotorPower, 0),.6); //movement safety
        }
        if(turretHomed && turretMotorPosition >14000){
            turretMotorPower = Math.min(Math.max(turretMotorPower, -0.6),0); //movement safety
        }
        turret.setPower(turretMotorPower);

        //shoulder movement
        if(armAutomation){
            shoulder_controller.setPID(shoulder_p, shoulder_i, shoulder_d);
            shoulderMotorPower = -shoulder_controller.calculate(armPos,shoulder_target);
        }
        shoulderMotorPower = Math.min(Math.max(shoulderMotorPower, -.2),.2); //movement safety
        if(armPos <.35){
            shoulderMotorPower = Math.min(Math.max(shoulderMotorPower, -.2),0); //movement safety
        }
        if(armPos > 2.437){
            shoulderMotorPower = Math.min(Math.max(shoulderMotorPower, 0),.2); //movement safety
        }
        shoulder.setPower(shoulderMotorPower);

        //pump motor
        pump.setPower(pumpMotorPower);

        //fan relay
        valve1.setState(fanRelay);

        //Fogger relay
        compressor1.setState(foggerRelay);
    }

    private void controllersGetValues (){
        //drive train controls
        rightdriveMotorPower = Math.min(Math.max(-gamepad1.left_stick_y * 0.3, -0.3), 0.3);
        leftdriveMotorPower = Math.min(Math.max(gamepad1.right_stick_y * 0.3, -0.3), 0.3);

        //automate spray
        if (gamepad1.x &&!ButtonX1block) { //moves arm to target positions
            ButtonX1block = true;
            automationState = 1;
            armAutomation = true;
            linkage_target = -100;
            shoulder_movement_target = 2;
            wrist_target = .7;
            turret_movement_target = 1500;
            automaticSpray();
        }
        else if (!gamepad1.x){
            ButtonX1block = false;
        }

        arm_to_AprilTag = gamepad1.b;//test arm movement based on apriltags

        if(gamepad1.dpad_left){//load test values for vision
            //Axes orientation: +x right, +y forward, +z upward
            aprilTag_Target_Left = new Position(DistanceUnit.INCH,-21, 13.5, -6,0);
            aprilTag_Target_Right = new Position(DistanceUnit.INCH,-20, 10, -4,0);
        }
        /*
        if(gamepad1.b && !ButtonB1block){ //test arm movement based on apriltags
            ButtonB1block = true;
            arm_to_AprilTag = true;
            automationTelemetryTest();
        }
        else if (!gamepad1.b){
            ButtonB1block = false;
        }*/
        if (gamepad1.y){ //turn off automation
            armAutomation = false;
            linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            automationState = 0;
        }
        if (gamepad1.a && !ButtonA1block){//stow turret
            ButtonA1block = true;
            armAutomation = true;
            automationState = 99;
        }
        else if(!gamepad1.a){
            ButtonA1block = false;
        }
        //pump controls
        if (gamepad2.y && !ButtonBlockValuePump) {
            ButtonBlockValuePump = true;
            if (pumpMotorPower == 0) {
                pumpMotorPower = -1;
            } else {
                pumpMotorPower = 0;
            }
        } else if (!gamepad2.y) {
            ButtonBlockValuePump = false;
        }
        if (gamepad2.a && !ButtonAblock) {
            ButtonAblock = true;
            if (pumpMotorPower == 0) {
                pumpMotorPower = 1;
            } else {
                pumpMotorPower = 0;
            }
        } else if (!gamepad2.a) {
            ButtonAblock = false;
        }

        //fan relay controls
        if (gamepad2.x && !ValveButtonBlock) {
            ValveButtonBlock = true;
            fanRelay = !fanRelay;
        } else if (!gamepad2.x) {
            ValveButtonBlock = false;
        }

        //fogger relay controls
        if (gamepad2.b && !CompressorButtonBlock) {
            CompressorButtonBlock = true;
            foggerRelay = !foggerRelay;
        } else if (!gamepad2.b) {
            CompressorButtonBlock = false;
        }

        //manual arm controls
        if (!armAutomation) {
            //linkage manual controls
            if (gamepad2.dpad_left) {
                linkageMotorPower = .5;
            } else if (gamepad2.dpad_right) {
                linkageMotorPower = -0.5;
            } else if (!gamepad1.dpad_up && !gamepad1.dpad_up) {
                linkageMotorPower = 0;
            }

            //wrist servo manual controls
            if (gamepad2.dpad_up) {
                wristServoPosition = Math.min(Math.max(wristServoPosition + 0.01, 0), 1);
            } else if (gamepad2.dpad_down) {
                wristServoPosition = Math.min(Math.max(wristServoPosition - 0.01, 0), 1);
            }

            //turret manual controls
            turretMotorPower = gamepad2.right_trigger - gamepad2.left_trigger;
            shoulderMotorPower = -gamepad2.right_stick_y;
        }
    }

    private void automaticSpray(){
        if(automationState ==0) {
            armAutomation = false;
        }
        else if(automationState == 1) {
            if(armStowed){
                automationState = 2;
            }
            else{
                automationState = 99;
            }
        }
        else if(automationState == 2){
            automationState = 3;
            wristServoPosition = wrist_target;
            turret_target = turret_movement_target;
            shoulder_target = shoulder_movement_target;
        }
        else if (automationState ==3){
            if((abs(turret_target - turretMotorPosition))<100  && armPos>shoulder_movement_target - .05 && armPos < shoulder_movement_target +.05){
                linkage.setTargetPosition(linkage_target);
                linkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linkageMotorPower = -.5;
                automationState = 4;
            }
        }
        else if (automationState ==4){
            if(!linkage.isBusy()){
                linkageMotorPower = 0;
                linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                automationState = 5;
            }
        }
        else if (automationState ==5) {
            if (!linkage.isBusy() && (abs(turret_target - turretMotorPosition)) < 100 && armPos > shoulder_movement_target - .05 && armPos < shoulder_movement_target + .05) {
                automationState = 0;
                armAutomation = false;
            }
        }
        else if(automationState == 99){
            automationState = 98;
            shoulder_target = armPos;
            turret_target = (int) turretMotorPosition;
            linkageMotorPower = 0.5;

        }
        else if(automationState == 98){
            if(!rearLinkageLimit){
                linkageMotorPower = 0;
                turret_target = 6000;
                wristServoPosition = .5;
                shoulder_target = shoulder_stowed;
                automationState = 97;
            }
        }
        else if(automationState == 97){
            if(!rearLinkageLimit && (abs(turret_target - turretMotorPosition))<50  && armPos> shoulder_stowed - .05 && armPos < shoulder_stowed +.05){
                armStowed = true;
                automationState = 0;
                armAutomation = false;
            }
        }
        else if (automationState ==50){
            if(!linkage.isBusy()) {
                armAutomation = false;
                automationState = 0;}
        }
        else{
            armAutomation = false;
        }
    }
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, Constants.cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, true);
    }   // end method initAprilTag()

    private Position getRobotRelativeCoordinate(Position p){
        Position coordsReoriented = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
    //since the camera is currently (6/24) only rotated in yaw, I've done this for now, but we should have a matrix implementation next
        coordsReoriented.x = p.x * Math.cos(-YAW_CAM)- p.y * Math.sin(-YAW_CAM);
        coordsReoriented.y = p.x * Math.sin(-YAW_CAM)+ p.y * Math.cos(-YAW_CAM);
	coordsReoriented.z = p.z;

        coordsReoriented.x = coordsReoriented.x + Constants.X_CAM;
        coordsReoriented.y = coordsReoriented.y + Constants.Y_CAM;
        coordsReoriented.z = coordsReoriented.z + Constants.Z_CAM;
        return coordsReoriented;
    }
private double[] getGeometricTargets(double x_robotrel, double y_robotrel, double z_robotrel) { // returns array containing turret angle (radians), extension distance (inches), arm angle (radians)
	double turret_angle = Math.atan2(y_robotrel, x_robotrel); //subtracting angle because theta is defined on the y-axis
	double how_far_extend = 0;
	double arm_angle_goal = 0;

	//what if the extension target is outside of its maximum length, and the arm can’t reach it?
	//then the arm should point to the QR code and extend to the maximum length
	if (Math.sqrt(Math.pow(x_robotrel, 2)+ Math.pow(y_robotrel, 2)) > MAX_EXTENSION_LENGTH + QR_distance_away && ARM_LENGTH < Math.sqrt((x_robotrel - (MAX_EXTENSION_LENGTH + QR_distance_away) * Math.pow(Math.cos(turret_angle), 2)) + (y_robotrel - (MAX_EXTENSION_LENGTH + QR_distance_away) * Math.pow(Math.sin(turret_angle),2 )+ Math.pow((z_robotrel - PIVOT_HEIGHT), 2)))) {

        	how_far_extend = EXTENSION_RANGE;
		double x_pivot = x_robotrel - MAX_EXTENSION_LENGTH * Math.cos(turret_angle);
		double y_pivot = y_robotrel - MAX_EXTENSION_LENGTH * Math.sin(turret_angle);
        	arm_angle_goal = Math.atan2(z_robotrel - PIVOT_HEIGHT, Math.pow(x_pivot, 2) + Math.pow(y_pivot,2));
	}
	//what if the target is in range of the extension, but out of range for the arm?
	//then the arm should point directly up or down, and the extension should go directly under or above the location
	else if ((z_robotrel - PIVOT_HEIGHT) / ARM_LENGTH > 1) {
        	arm_angle_goal = Math.PI / 2;
        	how_far_extend = Math.sqrt(Math.pow(x_robotrel,2) + Math.pow(y_robotrel, 2)) - MIN_EXTENSION_LENGTH;
	}
	//this one will likely never be used, and if the robot enters this state, something has gone wrong
	else if ((z_robotrel - PIVOT_HEIGHT) / ARM_LENGTH < -1) {
        	arm_angle_goal = -Math.PI / 2;
        	how_far_extend = Math.sqrt(Math.pow(x_robotrel,2) + Math.pow(y_robotrel, 2)) - MIN_EXTENSION_LENGTH;
	}

	//if the target is in-range of both the arm and extension, calculate normally
	else {
		arm_angle = Math.asin((z_robotrel - PIVOT_HEIGHT) / ARM_LENGTH);

        how_far_extend = Math.sqrt(Math.pow(x_robotrel,2) + Math.pow(y_robotrel, 2)) - ARM_LENGTH * cos(arm_angle) - MIN_EXTENSION_LENGTH - QR_distance_away;
		//distance from qr to center of turret MINUS horizontal distance of the arm MINUS length of retracted extension MINUS desired distance from QR code
	}

	return new double[]{turret_angle, how_far_extend, arm_angle_goal};
}

private int getExtensionEncoderTarget(boolean isOldArm, double length_extended) {
	//if we’re using the new arm, we can just convert inches to encoder ticks
	if (!isOldArm)
		return (int)(length_extended / EXTENSION_ENCODER_TO_INCHES);
	//if we’re using the old arm, we need to account for the linkage
	else {
		//horizontal distance between the motor and the link 2 attachment point
		double x_attach = length_extended + MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE + EXTENSION_MOTOR_OFFSET;
		double y_attach = LINK_2_ATTACHMENT_HEIGHT;
		
		double motor_angle = Math.abs(Math.asin((Math.pow(x_attach,2) + Math.pow(y_attach,2) + Math.pow(LINKAGE_LENGTH_1,2) - Math.pow(LINKAGE_LENGTH_2, 2)) / (2 * LINKAGE_LENGTH_1 * Math.sqrt(Math.pow(x_attach ,2) + Math.pow(y_attach, 2))) - Math.atan2(x_attach, y_attach)) - RETRACTED_LINK_1_ANGLE);
		//this value assumes 
		return (int)(motor_angle / EXTENSION_ENCODER_TO_RADIANS);
	}
}

//placeholder functions for moving the sprayer to position
private void rotateTurret(double angleTarget){}
private void extendArm(double distanceTarget){}
private void rotateArm(double angleTarget){}

private void automationTelemetryTest(){
        //currentAprilTag = currentDetections.get(0);
        //cameraRelCoords = currentAprilTag.robotPose.getPosition();
        cameraRelCoords = aprilTag_Target_Left;
        telemetry.addData("camera relative marker position: ", cameraRelCoords);
        robotRelCoords = getRobotRelativeCoordinate(cameraRelCoords);
        telemetry.addData("robot relative marker position: ", robotRelCoords);
        double[] target_values = getGeometricTargets(robotRelCoords.x, robotRelCoords.y, robotRelCoords.z);
        telemetry.addData("turret angle (deg): ", toDegrees(target_values[0]));
        telemetry.addData("extension length (in): ", target_values[1]);
        telemetry.addData("arm angle (deg): ", toDegrees(target_values[2]));
}
}
