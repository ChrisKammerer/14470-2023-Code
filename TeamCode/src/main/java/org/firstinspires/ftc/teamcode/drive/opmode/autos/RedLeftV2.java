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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class RedLeftV2 extends LinearOpMode {
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
    final int ARM_2 = 244;
    final int ARM_3 = 376;
    final int ARM_4 = 540;
    final int ARM_5 = 667;
    final int ARM_UP_FRONT = 875;
    final int ARM_MID = 1340;
    final int ARM_BACK_POLE = 1956;
    final int ARM_BACK_FULL = 2500;

    final double LCLAW_MID = .64;
    final double RCLAW_MID = .16;
    final double LCLAW_OPEN = .75;
    final double RCLAW_OPEN = .05;
    final double LCLAW_CLOSE = .55;
    final double RCLAW_CLOSE = .24;


    private ElapsedTime runtime = new ElapsedTime();

    enum State{
        TRAJECTORY_1, // move forward & spline
        TRAJECTORY_2, // back up and turn
        TRAJECTORY_3, // move to cones
        TRAJECTORY_4, // back up to pole
        TRAJECTORY_5, // move to cones 2
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
                .lineToConstantHeading(new Vector2d(40, 0))
                .splineTo(new Vector2d(46.5,-8.5), 5.45)
                .build();
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(43, -2, Math.toRadians(90)))
                .build();
        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToConstantHeading(new Vector2d(44, 19.6))
                .build();
        Trajectory trajectory4 = drive.trajectoryBuilder(trajectory3.end(), true)
                .lineToConstantHeading(new Vector2d(44, 2))
                .build();

        //TODO: split trajectory 1 into 2 trajectories, and replace the spline with a lineToLinearHeading
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
                    if(runtime.seconds()>1&&runtime.seconds()<1.2){
                        chainBar.setTargetPosition(ARM_UP_FRONT);
                        chainBar.setPower(.5);
                        leftWinch.setTargetPosition(LIFT_HIGH);
                        rightWinch.setTargetPosition(LIFT_HIGH);
                        leftWinch.setPower(1);
                        rightWinch.setPower(1);
                    }
                    if(runtime.seconds()>3.2&&runtime.seconds()<3.4){
                        chainBar.setTargetPosition(ARM_MID);
                        leftWinch.setTargetPosition(LIFT_BOTTOM);
                        rightWinch.setTargetPosition(LIFT_BOTTOM);
                    }
                    if(runtime.seconds()>3.4&&runtime.seconds()<3.6){
                        leftIntake.setPosition(LCLAW_MID);
                        rightIntake.setPosition(RCLAW_MID);

                    }
                    if(!drive.isBusy()&&runtime.seconds()>4.5){
                        currentState = State.TRAJECTORY_2;
                        chainBar.setTargetPosition(ARM_MID);
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
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
                        drive.followTrajectoryAsync(trajectory4);
                        currentState = State.IDLE;
                    }
                    break;
                case TRAJECTORY_4:
                    if(!drive.isBusy()){
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