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
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Position cameraPosition = new Position(DistanceUnit.INCH,-8, -7, 13, 0);
    Position cameraRelCoords;
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,45, -90, 0, 0);
    //Camera Pose
    double X_CAM = -14.5, Y_CAM = -9, Z_CAM= 17;
    double PITCH_CAM = 0.785398, YAW_CAM = -1.5708, ROLL_CAM = 0;
    //extension measurements
    double MIN_EXTENSION_LENGTH = 10.25;
    double EXTENSION_RANGE = 13.75;
    double MAX_EXTENSION_LENGTH = MIN_EXTENSION_LENGTH + EXTENSION_RANGE;
    //arm or shoulder measurements
    double PIVOT_HEIGHT = 3.5;
    double z_0 = X_CAM - PIVOT_HEIGHT;
    double ARM_LENGTH = 16.5;
    double LINKAGE_LENGTH_1 = 12;
    double LINKAGE_LENGTH_2 = 13;
    double SLIDER_HEIGHT = 1.7;
    double TIP_TO_PIVOT_DISTANCE = 8;
    double RETRACTED_LINK_1_ANGLE;
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

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        initAprilTag();
        rotation_matrix = new Position (DistanceUnit.INCH,-8, -7, 13, 0);
    //[[cos(YAW_CAM)*cos(PITCH_CAM)], [sin(YAW_CAM)*sin(PITCH_CAM)*sin(ROLL_CAM) + cos(YAW_CAM)*cos(ROLL_CAM)], [sin(YAW_CAM)*sin(PITCH_CAM)*cos(ROLL_CAM) - cos(YAW_CAM)*sin(ROLL_CAM)]], [[sin(YAW_CAM)*cos(PITCH_CAM)], [sin(YAW_CAM)*sin(PITCH_CAM)*sin(ROLL_CAM) + cos(YAW_CAM)*cos(ROLL_CAM)], [sin(YAW_CAM)*sin(PITCH_CAM)*cos(ROLL_CAM) - cos(YAW_CAM)*sin(ROLL_CAM)]], [[-sin(PITCH_CAM)], [cos(PITCH_CAM)*sin(ROLL_CAM)], [cos(PITCH_CAM)*cos(ROLL_CAM)]];
        RETRACTED_LINK_1_ANGLE = asin(pow((MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE),2) + pow(SLIDER_HEIGHT,2)+ pow(LINKAGE_LENGTH_1,2)-pow(LINKAGE_LENGTH_2,2)) / (2* LINKAGE_LENGTH_1 * sqrt(pow((MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE),2)+ pow(SLIDER_HEIGHT,2))) - atan2((MIN_EXTENSION_LENGTH - TIP_TO_PIVOT_DISTANCE) , SLIDER_HEIGHT);	//radians

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
            robotRelCoords = getRobotRelativeCoordinate (cameraRelCoords);
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
        telemetry.addLine(String.format("Auto: State %d Enabled %b Stowed %b Homed %b",automationState, armAutomation, armStowed, turretHomed));
        telemetry.addLine(String.format("Link: Pos %d Front %b Rear %b Busy %b",(int) linkageMotorPosition,frontLinkageLimit, rearLinkageLimit, linkage.isBusy()));
        telemetry.addLine(String.format("Fogger(False on): Fan %b Fog %b Pump (%.2f)", foggerRelay, fanRelay, pumpMotorPower));
        telemetry.addLine(String.format("Arm Loc Turret: Pos %d Target %d Angle (%.1f)",(int)turretMotorPosition, turret_target, (turretMotorPosition/turret_ticks_in_degree)-93));
        telemetry.addLine(String.format("Arm Loc Wrist: (%.2f) Shoulder: Raw (%.2f), Angle (%.1f)", wristServoPosition, armPos, shoulder_angle));
        telemetry.addLine(String.format("Status: Run Time: %s Average Loop: %.0f", runtime.toString(), averageLoopTime));
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftdriveMotorPower, rightdriveMotorPower);
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y, detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES), detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
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
    private void sensorRead (){
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
        if (!currentDetections.isEmpty()) {
            AprilTagDetection detection = currentDetections.get(0);
            detectedTageID = detection.id;
            detectedPosition = new Position (DistanceUnit.INCH, detection.robotPose.getPosition().x, detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z, 0);
            detectedYPRA = new YawPitchRollAngles(AngleUnit.DEGREES, detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES), detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES),
                    0);
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
        if (gamepad1.x &&!ButtonX1block) {
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
        if(gamepad1.b && !ButtonB1block){
            ButtonB1block = true;
            linkageMotorPower = 1;
            armAutomation = true;
            linkage.setTargetPosition(-100);
            linkage.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            automationState = 50;
        }
        else if (!gamepad1.b){
            ButtonB1block = false;
        }
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
                .setCameraPose(cameraPosition, cameraOrientation)

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
        aprilTag.setDecimation(1);

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
        Position coordsReoriented = p;//rotation_matrix * p;
        coordsReoriented.x = coordsReoriented.x - X_CAM;
        coordsReoriented.y = coordsReoriented.y - Y_CAM;
        coordsReoriented.z = coordsReoriented.z - Z_CAM;
        return coordsReoriented;
    }
}
