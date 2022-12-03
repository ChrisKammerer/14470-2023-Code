package org.firstinspires.ftc.teamcode.drive.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class GigaRigged3Cone extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 0;
    int MIDDLE = 1;
    int RIGHT = 2;
    AprilTagDetection tagOfInterest = null;
    DcMotor leftWinch;
    DcMotor rightWinch;
    DcMotor turret;
    DcMotor chainBar;
    Servo leftIntake;
    Servo rightIntake;
    final int LIFT_LOW = 0;
    final int LIFT_LOW2 = 1200;
    final int LIFT_MID = 2950;
    final int LIFT_HIGH = 4640;
    final int ARM_LOW = 0;
    final int ARM_MID = 920;
    final int ARM_HIGH = 1400;
    final int ARM_BACK = -200;
    final int TURRET_RIGHT = -1185;
    final int TURRET_CENTER = 0;

    final int ARM_5CONE = 690;
    final int ARM_4CONE = 620;


    private ElapsedTime runtime = new ElapsedTime();

    enum State{
        TRAJECTORY_1, // move forward
        TRAJECTORY_2, // move to mid pole and raise lift
        WAIT_1, // drop cone and lift
        TRAJECTORY_3, // strafe left
        TRAJECTORY_4, // forward to cone stack
        TRAJECTORY_4_5,
        TURN_1, // turn to face cone stack
        TRAJECTORY_5, // drive to cone stack
        WAIT_2, // pick up cone1
        TRAJECTORY_6, // drive away from stack a bit
        TURN_2, // Turn to face pole
        TRAJECTORY_7, //drive to high pole and raise lift
        WAIT_3, //drop cones and lower lift
        TRAJECTORY_7_5, //return to pre placement position
        TURN_3, //turn to face cone stack
        TRAJECTORY_8, //move to cone stack
        WAIT_4, // pick up cone and raise lift
        TRAJECTORY_9, // move to short pole y
        TRAJECTORY_10, //move to short pole x
        WAIT_5, // drop cone
        TRAJECTORY_11, // move away from short pole

        TRAJECTORY_PARK, //park

        IDLE //end
    }
    State currentState = State.IDLE;

    @Override
    public void runOpMode() {
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        turret = hardwareMap.dcMotor.get("turret");
        chainBar = hardwareMap.dcMotor.get("chainBar");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(TURRET_CENTER);
        rightWinch.setTargetPosition(LIFT_LOW);
        leftWinch.setTargetPosition(LIFT_LOW);
        chainBar.setTargetPosition(ARM_LOW);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        //CREATE TRAJECTORIES HERE
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addData("Tag found", tagOfInterest.id);
                } else {
                    telemetry.addLine("Tag not found");
                }

            }
            telemetry.update();
            sleep(20);
        }

        // Trajectory 1: move forward
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(26, 0))
                .build();

        // Trajectory 2: move to the base of the mid height pole
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineTo(new Vector2d(27.5, -10.2))
                .build();
        // Trajectory 3: return to center lane
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineTo(new Vector2d(28, 1))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end())
                .lineTo(new Vector2d(55, 2.6))
                .build();
        // Trajectory 4: move forward to cone x value
        Trajectory trajectory4_5 = drive.trajectoryBuilder(trajectory4.end())
                .lineTo(new Vector2d(50.6, 2.6))
                .build();
        // Trajectory 5: drive left and position to grab cone
        Trajectory trajectory5 = drive.trajectoryBuilder(trajectory4_5.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .lineTo(new Vector2d(47.5, 26.4))
                .build();
        // Trajectory 6: drive backwards
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .lineToConstantHeading(new Vector2d(49, 7))
                .build();
        //Trajectory 7: move to cone placement
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end().plus(new Pose2d(0,0,Math.toRadians(-45))))
                .lineToConstantHeading(new Vector2d(53, 2.1))
                .build();
        //Trajectory 7_5: move back to pre placement position
        Trajectory trajectory7_5 = drive.trajectoryBuilder(trajectory7.end())
                .lineToConstantHeading(new Vector2d(49, 7))
                .build();
        //Trajectory 8: move to stack again
        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7_5.end().plus(new Pose2d(0,0,Math.toRadians(45))))
                .lineTo(new Vector2d(47.5,25.7))
                .build();
        //Trajectory 9: move to pole y position
        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory8.end())
                .lineTo(new Vector2d(48, 3.85))
                .build();
        //Trajectory 10: move to pole x position
        Trajectory trajectory10 = drive.trajectoryBuilder(trajectory9.end())
                .lineTo(new Vector2d(37.2, 4.5))
                .build();
        //Trajectory 10: move to center
        Trajectory trajectory11 = drive.trajectoryBuilder(trajectory10.end())
                .lineTo(new Vector2d(49,4))
                .build();

        double parkX = 0;
        double parkY = 0;

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            parkX = 48.5;
            parkY = 25.6;
        } else if (tagOfInterest.id == MIDDLE) {
            parkX = 49;
            parkY = 3.2;
        } else if (tagOfInterest.id == RIGHT) {
            parkX = 52;
            parkY = -21;
        }
        // Trajectory Park: move to the correct parking space
        Trajectory trajectoryPark = drive.trajectoryBuilder(trajectory11.end())
                .lineTo(new Vector2d(parkX, parkY))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            switch (currentState) {
                case TRAJECTORY_1:
                    leftIntake.setPosition(.5);
                    rightIntake.setPosition(.3);
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                        leftWinch.setTargetPosition(LIFT_MID+400);
                        rightWinch.setTargetPosition(LIFT_MID+400);
                        chainBar.setTargetPosition(ARM_MID);
                        leftWinch.setPower(.75);
                        rightWinch.setPower(.75);
                        chainBar.setPower(.75);
                        turret.setTargetPosition(67);
                        turret.setPower(.5);
                    }
                    break;

                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;
                        runtime.reset();
                    }
                    break;
                case WAIT_1:
                    if(runtime.seconds()>1.5) {
                        leftWinch.setTargetPosition(LIFT_LOW);
                        rightWinch.setTargetPosition(LIFT_LOW);
                    }
                    if(runtime.seconds()>2){
                        leftIntake.setPosition(.75);
                        rightIntake.setPosition(0.05);
                        chainBar.setTargetPosition(ARM_LOW);
                    }
                    if(runtime.seconds()>3) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;

                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4;
                        drive.followTrajectoryAsync(trajectory4);
                    }
                    break;
                case TRAJECTORY_4:
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_4_5;
                        drive.followTrajectoryAsync(trajectory4_5);
                    }
                    break;
                case TRAJECTORY_4_5:
                    if(!drive.isBusy()){
                        currentState = State.TURN_1;
                        drive.turnAsync(Math.toRadians(90));
                    }
                case TURN_1:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_5;
                        chainBar.setTargetPosition(ARM_5CONE);
                        drive.followTrajectoryAsync(trajectory5);
                    }
                    break;
                case TRAJECTORY_5:
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_2;
                        runtime.reset();
                    }
                    break;
                case WAIT_2:
                    //close claw, wait, raise, wait, move
                    leftIntake.setPosition(.5);
                    rightIntake.setPosition(.3);
                    if(runtime.seconds()>1){
                        leftWinch.setTargetPosition(LIFT_LOW2);
                        rightWinch.setTargetPosition(LIFT_LOW2);
                        chainBar.setTargetPosition(ARM_HIGH);
                    }
                    if(runtime.seconds()>1.2){
                        turret.setTargetPosition(TURRET_RIGHT);
                        turret.setPower(.5);
                    }
                    if(runtime.seconds()>1.25){
                        currentState = State.TRAJECTORY_6;
                        drive.followTrajectoryAsync(trajectory6);
                    }
                    break;
                case TRAJECTORY_6:
                    if(!drive.isBusy()){
                        currentState = State.TURN_2;
                    }
                    break;
                case TURN_2:
                    if(!drive.isBusy()){
                        drive.turnAsync(Math.toRadians(-45));
                        drive.followTrajectory(trajectory7);
                        currentState = State.TRAJECTORY_7;
                    }
                    break;
                case TRAJECTORY_7:
                    if(!drive.isBusy()){
                        currentState = State.WAIT_3;
                        runtime.reset();
                    }
                    break;
                case WAIT_3:
                    if(runtime.seconds()<2.5){
                        leftWinch.setTargetPosition(LIFT_HIGH);
                        rightWinch.setTargetPosition(LIFT_HIGH);
                        leftWinch.setPower(1);
                        rightWinch.setPower(1);
                    }
                    if(runtime.seconds()>2.5) {

                        leftWinch.setTargetPosition(LIFT_LOW);
                        rightWinch.setTargetPosition(LIFT_LOW);
                    }
                    if(runtime.seconds()>3.1){
                        rightIntake.setPosition(0.05);
                        leftIntake.setPosition(.75);
                    }
                    if(runtime.seconds()>3.15){
                        currentState = State.TRAJECTORY_7_5;
                        drive.followTrajectoryAsync(trajectory7_5);
                        turret.setTargetPosition(TURRET_CENTER);
                    }
                    break;
                case TRAJECTORY_7_5:
                    if(!drive.isBusy()) {
                        currentState = State.TURN_3;
                    }
                    break;
                case TURN_3:
                    if(!drive.isBusy()) {
                        drive.turnAsync(Math.toRadians(45));
                        drive.followTrajectoryAsync(trajectory8);
                        chainBar.setTargetPosition(ARM_4CONE);
                        currentState = State.TRAJECTORY_8;
                        runtime.reset();
                    }
                    break;
                case TRAJECTORY_8:
                    if(!drive.isBusy()) {
                        currentState = State.WAIT_4;
                        runtime.reset();
                    }
                case WAIT_4:
                    if(runtime.seconds()>1.1){
                        leftIntake.setPosition(.5);
                        rightIntake.setPosition(.3);
                    }
                    if(runtime.seconds()>1.3){
                        leftWinch.setTargetPosition(LIFT_LOW2);
                        rightWinch.setTargetPosition(LIFT_LOW2);
                        chainBar.setTargetPosition(ARM_HIGH);
                    }
                    if(runtime.seconds()>1.4){
                        drive.followTrajectoryAsync(trajectory9);
                        currentState = State.TRAJECTORY_9;
                        runtime.reset();
                    }
                    break;
                case TRAJECTORY_9:
                    if(!drive.isBusy()) {
                        drive.followTrajectoryAsync(trajectory10);
                        currentState = State.TRAJECTORY_10;
                    }
                    break;
                case TRAJECTORY_10:
                    if(!drive.isBusy()) {
                        currentState = State.WAIT_5;
                        runtime.reset();
                    }
                    break;
                case WAIT_5:
                    if(runtime.seconds()>.3){
                        leftWinch.setTargetPosition(LIFT_LOW);
                        rightWinch.setTargetPosition(LIFT_LOW);
                    }
                    if(runtime.seconds()>.5){
                        rightIntake.setPosition(0.05);
                        leftIntake.setPosition(.75);
                    }
                    if(runtime.seconds()>.55){
                        drive.followTrajectoryAsync(trajectory11);
                        currentState = State.TRAJECTORY_11;
                    }
                    break;
                case TRAJECTORY_11:
                    if(!drive.isBusy()){
                        chainBar.setTargetPosition(ARM_LOW);
                        currentState = State.TRAJECTORY_PARK;
                        drive.followTrajectoryAsync(trajectoryPark);
                        runtime.reset();
                    }
                    break;
                case TRAJECTORY_PARK:
                    if(!drive.isBusy()){
                        rightIntake.setPosition(0.05);
                        leftIntake.setPosition(.81);
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            drive.update();
            telemetry.addData("state", currentState);
            telemetry.update();
        }
    }
}