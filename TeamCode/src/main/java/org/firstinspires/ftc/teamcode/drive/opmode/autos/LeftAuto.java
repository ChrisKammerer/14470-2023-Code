package org.firstinspires.ftc.teamcode.drive.opmode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import java.util.ArrayList;

@Autonomous
public class LeftAuto extends LinearOpMode {
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
    final int LIFT_MID = 3100;
    final int LIFT_HIGH = 4300;
    final int ARM_LOW = 0;
    final int ARM_MID = 40;
    final int ARM_HIGH = 1070;
    final int ARM_BACK = -200;
    private ElapsedTime runtime = new ElapsedTime();

    enum State{
        TRAJECTORY_1, // move forward
        TRAJECTORY_2, // move to mid pole and raise lift
        WAIT_1, // drop cone and lift
        TRAJECTORY_3, // park
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
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setTargetPosition(LIFT_LOW);
        leftWinch.setTargetPosition(LIFT_LOW);
        chainBar.setTargetPosition(ARM_LOW);
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
        //CREATE TRAJECTORIES
        currentState = State.TRAJECTORY_1;
        //follow trajectory_1

        switch(currentState){
            case TRAJECTORY_1:
                if(!drive.isBusy()){
                    currentState = State.TRAJECTORY_2;
                }
        }


    }
}

