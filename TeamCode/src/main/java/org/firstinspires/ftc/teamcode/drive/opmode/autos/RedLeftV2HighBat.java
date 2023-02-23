package org.firstinspires.ftc.teamcode.drive.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RedLeftV2HighBat extends LinearOpMode {
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
    DcMotor chainBar;
    Servo leftIntake;
    Servo rightIntake;
    final int LIFT_BOTTOM = 0;
    final int LIFT_LOW = 850;
    final int LIFT_MID = 2150;
    final int LIFT_HIGH = 3375;

    final int ARM_0 = 0;
    final int ARM_1 = 80;
    final int ARM_2 = 224;
    final int ARM_3 = 426;
    final int ARM_4 = 550;
    final int ARM_5 = 670;
    final int ARM_UP_FRONT = 875;
    final int ARM_MID = 1340;
    final int ARM_BACK_POLE = 2650;
    final int ARM_BACK_FULL = 2500;

    final double LCLAW_MID = .64;
    final double RCLAW_MID = .16;
    final double LCLAW_OPEN = .75;
    final double RCLAW_OPEN = .05;
    final double LCLAW_CLOSE = .54;
    final double RCLAW_CLOSE = .25;


    private ElapsedTime runtime = new ElapsedTime();

    enum State{
        TRAJECTORY_1, // move forward & spline
        TRAJECTORY_1_5,
        TRAJECTORY_1_5_2,
        TRAJECTORY_2, // back up and turn
        TRAJECTORY_3, // move to cones
        TRAJECTORY_4, // back up to pole
        TRAJECTORY_5, // move to cones 2
        TRAJECTORY_6,
        TRAJECTORY_7,
        TRAJECTORY_8,
        TRAJECTORY_9,
        IDLE, //end
    }
    State currentState = State.IDLE;

    @Override
    public void runOpMode() {
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");

        chainBar = hardwareMap.dcMotor.get("chainBar");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");

        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        rightWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightWinch.setTargetPosition(LIFT_BOTTOM);
        leftWinch.setTargetPosition(LIFT_BOTTOM);
        chainBar.setTargetPosition(ARM_0);

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


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
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

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(47.1, 0))
//                .splineTo(new Vector2d(46.6,-8.1), 5.45)
                .build();
        Trajectory trajectory1_5 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(47.4, -7.8, Math.toRadians(-25)))
                .build();
        Trajectory trajectory1_5_2 = drive.trajectoryBuilder(trajectory1_5.end(), true)
                .lineToConstantHeading(new Vector2d(45.1, -5.7))
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1_5_2.end())
                .lineToLinearHeading(new Pose2d(46.5, -6, Math.toRadians(87)), //og 45.5
                    drive.getVelocityConstraint(DriveConstants.MAX_VEL, Math.toRadians(140), DriveConstants.TRACK_WIDTH),
                    drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(50.1, 16.9, Math.toRadians(90)))
                .build();
//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
//                .lineToLinearHeading(new Pose2d(50, 0, Math.toRadians(145)))
//                .build();
        Trajectory trajectory5 = drive.trajectoryBuilder((trajectory3.end()), true)
                .splineTo(new Vector2d(59.3, -7), Math.toRadians(165+180))
                .build();
//        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
//                .lineToConstantHeading(new Vector2d(49, 6))
//                .splineTo(new Vector2d(57, -8.5), Math.toRadians(145))
//                .build();
//        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
//                .lineToLinearHeading(new Pose2d(54, 0, Math.toRadians(90)))
//                .splineTo(new Vector2d(50, 18), Math.toRadians(90))
//                .build();
        Trajectory trajectory6 = drive.trajectoryBuilder(trajectory5.end())
                .splineTo(new Vector2d(52, 17.8), Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(50, 18))
                .build();
        Trajectory trajectory7 = drive.trajectoryBuilder(trajectory6.end(), true)
                .splineTo(new Vector2d(59.3, -6.85), Math.toRadians(165+180))
                .build();
//        Trajectory trajectory8 = drive.trajectoryBuilder(trajectory7.end())
//                .splineTo(new Vector2d(50.5, 18.2), Math.toRadians(90))
//                .build();
//        Trajectory trajectory9 = drive.trajectoryBuilder(trajectory6.end(), true)
//                .splineTo(new Vector2d(59.3, -6.6), Math.toRadians(165+180))
//                .build();

        double parkX = 0;
        double parkY = 0;

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            parkX = 0;
            parkY = 0;
        } else if (tagOfInterest.id == MIDDLE) {
            parkX = 0;
            parkY = 0;
        } else if (tagOfInterest.id == RIGHT) {
            parkX = 0;
            parkY = 0;
        }
        // Trajectory Park: move to the correct parking space
        // PUT TRAJ HERE

        waitForStart();
        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        runtime.reset();
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            //debug stuff
//            if(runtime.seconds()>2&&runtime.seconds()<2.2){
//                leftWinch.setTargetPosition(LIFT_HIGH);
//                rightWinch.setTargetPosition(LIFT_HIGH);
//                leftWinch.setPower(1);
//                rightWinch.setPower(1);
//            }
//            if(runtime.seconds()>6&&runtime.seconds()<6.2){
//                leftWinch.setTargetPosition(LIFT_BOTTOM);
//                rightWinch.setTargetPosition(LIFT_BOTTOM);
//            }


            switch (currentState) {
                case TRAJECTORY_1:
                    if(runtime.seconds()>0&&runtime.seconds()<1) {
                        leftIntake.setPosition(LCLAW_CLOSE);
                        rightIntake.setPosition(RCLAW_CLOSE);
                    }
                    if(runtime.seconds()>.3&&runtime.seconds()<.5){
                        chainBar.setTargetPosition(ARM_MID);
                        chainBar.setPower(1);
                    }
                    if(!drive.isBusy()) {
                        runtime.reset();
                        currentState = State.TRAJECTORY_1_5;
                        drive.followTrajectoryAsync(trajectory1_5);
                    }
                    break;
                case TRAJECTORY_1_5:
                    if(runtime.seconds()>1&&runtime.seconds()<1.2){

                        leftWinch.setTargetPosition(LIFT_HIGH);
                        rightWinch.setTargetPosition(LIFT_HIGH);
                        leftWinch.setPower(1);
                        rightWinch.setPower(1);
                    }
                    if(runtime.seconds()>2.5&&runtime.seconds()<2.7){
                        chainBar.setTargetPosition(ARM_5);
                    }
                    if(runtime.seconds()>3&&runtime.seconds()<3.2){
                        leftIntake.setPosition(LCLAW_MID);
                        rightIntake.setPosition(RCLAW_MID);

                    }
                    if(runtime.seconds()>3.4&&runtime.seconds()<3.6){
                        chainBar.setTargetPosition(ARM_MID);
                        leftWinch.setTargetPosition(LIFT_BOTTOM);
                        rightWinch.setTargetPosition(LIFT_BOTTOM);
                    }
                    if(!drive.isBusy()&&runtime.seconds()>3.7){
                        currentState = State.TRAJECTORY_1_5_2;
                        chainBar.setTargetPosition(ARM_MID);
                        drive.followTrajectoryAsync(trajectory1_5_2);
                    }
                    break;
                case TRAJECTORY_1_5_2:
                    if(!drive.isBusy()){
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                case TRAJECTORY_2: // turn around to aim at cones
                    if(!drive.isBusy()){
                        drive.followTrajectoryAsync(trajectory3);
                        chainBar.setTargetPosition(ARM_5);
                        currentState = State.TRAJECTORY_3;
                        runtime.reset();
                    }
                    break;
                case TRAJECTORY_3: // go to cone stack
                    if(runtime.seconds()>2&&runtime.seconds()<2.2){
                        leftIntake.setPosition(LCLAW_CLOSE);
                        rightIntake.setPosition(RCLAW_CLOSE);

                    }
                    if(runtime.seconds()>2.2&&runtime.seconds()<2.4){
                        leftWinch.setTargetPosition(LIFT_LOW);
                        rightWinch.setTargetPosition(LIFT_LOW);
                        chainBar.setTargetPosition(ARM_MID);
                    }
                    if(!drive.isBusy()&&runtime.seconds()>2.7){
                        drive.followTrajectoryAsync(trajectory5);
                        runtime.reset();
                        currentState = State.TRAJECTORY_5;
                    }
                    break;
                case TRAJECTORY_5:
                    if(runtime.seconds()>1&&runtime.seconds()<1.2){
                        leftWinch.setTargetPosition(LIFT_HIGH);
                        rightWinch.setTargetPosition(LIFT_HIGH);
                    }
                    if(runtime.seconds()>2.2&&runtime.seconds()<2.4){
                        chainBar.setTargetPosition(ARM_BACK_POLE);
                    }
                    if(runtime.seconds()>2.9&&runtime.seconds()<3.1){
                        leftIntake.setPosition(LCLAW_MID);
                        rightIntake.setPosition(RCLAW_MID);
                        chainBar.setTargetPosition(ARM_4);
                    }
                    if(!drive.isBusy()&&runtime.seconds()>3.5){
                        drive.followTrajectoryAsync(trajectory6);
                        leftWinch.setTargetPosition(LIFT_BOTTOM);
                        rightWinch.setTargetPosition(LIFT_BOTTOM);
                        currentState = State.TRAJECTORY_6;
                        runtime.reset();
                    }
                    break;
                case TRAJECTORY_6:
                    if(runtime.seconds()>2&&runtime.seconds()<2.2){
                        leftIntake.setPosition(LCLAW_CLOSE);
                        rightIntake.setPosition(RCLAW_CLOSE);
                    }
                    if(runtime.seconds()>2.5&&runtime.seconds()<2.7){
                        chainBar.setTargetPosition(ARM_MID);
                        leftWinch.setTargetPosition(LIFT_LOW);
                        rightWinch.setTargetPosition(LIFT_LOW);
                    }
                    if(!drive.isBusy()&&runtime.seconds()>2.7){
                        drive.followTrajectoryAsync(trajectory7);
                        currentState = State.TRAJECTORY_7;
                        runtime.reset();
                    }
                    break;
                case TRAJECTORY_7:
                    if(runtime.seconds()>1&&runtime.seconds()<1.2){
                        leftWinch.setTargetPosition(LIFT_HIGH);
                        rightWinch.setTargetPosition(LIFT_HIGH);
                    }
                    if(runtime.seconds()>2&&runtime.seconds()<2.2){
                        chainBar.setTargetPosition(ARM_BACK_POLE);
                    }
                    if(runtime.seconds()>2.9&&runtime.seconds()<3.1){
                        leftIntake.setPosition(LCLAW_MID);
                        rightIntake.setPosition(RCLAW_MID);
                    }
                    if(runtime.seconds()>2.9&&runtime.seconds()<3.1){
                        chainBar.setTargetPosition(ARM_3);
                    }
                    if(!drive.isBusy()&&runtime.seconds()>3.4){
                        leftWinch.setTargetPosition(LIFT_BOTTOM);
                        rightWinch.setTargetPosition(LIFT_BOTTOM);
//                        drive.followTrajectoryAsync(trajectory8);
                        runtime.reset();
                        currentState = State.IDLE;
                    }
                    break;
//                case TRAJECTORY_8:
//                    if(runtime.seconds()>1.5&&runtime.seconds()>1.7){
//                        leftIntake.setPosition(LCLAW_CLOSE);
//                        rightIntake.setPosition(RCLAW_CLOSE);
//                    }
//                    if(runtime.seconds()>1.6&&runtime.seconds()>1.8){
//                        chainBar.setTargetPosition(ARM_MID);
//                        leftWinch.setTargetPosition(LIFT_LOW);
//                        rightWinch.setTargetPosition(LIFT_LOW);
//                    }
//                    if(!drive.isBusy()&& runtime.seconds()>2){
//                        drive.followTrajectoryAsync(trajectory9);
//                        runtime.reset();
//                        currentState = State.TRAJECTORY_9;
//                    }
//                    break;
//                case TRAJECTORY_9:
//                    if(runtime.seconds()>1&&runtime.seconds()<1.2){
//                        leftWinch.setTargetPosition(LIFT_HIGH);
//                        rightWinch.setTargetPosition(LIFT_HIGH);
//                    }
//                    if(runtime.seconds()>2&&runtime.seconds()>2.2){
//                        chainBar.setTargetPosition(ARM_BACK_POLE);
//                    }
//                    if(runtime.seconds()>2.8&&runtime.seconds()>3){
//                        leftIntake.setPosition(LCLAW_MID);
//                        rightIntake.setPosition(RCLAW_MID);
//                        chainBar.setTargetPosition(ARM_2);
//                    }
//                    if(!drive.isBusy()&&runtime.seconds()>3){
//                        currentState = State.IDLE;
//                    }
//                    break;
                case IDLE:
                    break;

            }

            drive.update();
            telemetry.addData("state", currentState);
            telemetry.update();
        }
    }
}