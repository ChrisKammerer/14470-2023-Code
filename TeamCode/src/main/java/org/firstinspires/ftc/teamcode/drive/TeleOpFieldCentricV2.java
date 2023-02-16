package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "drive")
public class TeleOpFieldCentricV2 extends LinearOpMode {

    DcMotor leftWinch;
    DcMotor rightWinch;
    DcMotor chainBar;
    Servo leftIntake;
    Servo rightIntake;

    double subtractHeading = 0;
    //final double CHAIN_BAR_DELAY_TIME = 0.0;
    //private ElapsedTime liftTimer = new ElapsedTime();


    final int LIFT_BOTTOM = 0;
    final int LIFT_LOW = 850;
    final int LIFT_MID = 2150;
    final int LIFT_HIGH = 3375;

    final int ARM_0 = 0;
    final int ARM_1 = 138;
    final int ARM_2 = 244;
    final int ARM_3 = 376;
    final int ARM_4 = 540;
    final int ARM_5 = 667;
    final int ARM_MID = 1340;
    final int ARM_BACK_POLE = 1956;
    final int ARM_BACK_FULL = 2500;

    final double LCLAW_MID = .64;
    final double RCLAW_MID = .16;
    final double LCLAW_OPEN = .75;
    final double RCLAW_OPEN = .05;
    final double LCLAW_CLOSE = .55;
    final double RCLAW_CLOSE = .24;


    int liftGoal = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        //turret = hardwareMap.dcMotor.get("turret");
        chainBar = hardwareMap.dcMotor.get("chainBar");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        leftIntake.setPosition(LCLAW_CLOSE);
        rightIntake.setPosition(RCLAW_CLOSE);

        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        rightWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightWinch.setTargetPosition(LIFT_BOTTOM);
        leftWinch.setTargetPosition(LIFT_BOTTOM);
//        chainBar.setTargetPosition(ARM_0);
        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        chainBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftTimer.reset();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(drive.getPoseEstimate());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.dpad_up){
                chainBar.setTargetPosition(ARM_BACK_POLE);
                chainBar.setPower(1);
            }
            if(gamepad1.dpad_down){
                chainBar.setTargetPosition(ARM_0);
                chainBar.setPower(1);
            }

            chainBar.setPower(gamepad1.right_trigger);
            if(gamepad1.left_trigger!=0) {
                chainBar.setPower(-gamepad1.left_trigger);
            }


            if(gamepad2.a){
                leftWinch.setTargetPosition(LIFT_LOW);
                rightWinch.setTargetPosition(LIFT_LOW);
                leftWinch.setPower(1);
                rightWinch.setPower(1);
            }
            if(gamepad2.b){
                leftWinch.setTargetPosition(LIFT_MID);
                rightWinch.setTargetPosition(LIFT_MID);
                leftWinch.setPower(1);
                rightWinch.setPower(1);
            }
            if(gamepad2.y){
                leftWinch.setTargetPosition(LIFT_HIGH);
                rightWinch.setTargetPosition(LIFT_HIGH);
                leftWinch.setPower(1);
                rightWinch.setPower(1);
            }
            if(gamepad2.a){
                leftWinch.setTargetPosition(LIFT_LOW);
                rightWinch.setTargetPosition(LIFT_LOW);
                leftWinch.setPower(1);
                rightWinch.setPower(1);
            }
            if(gamepad2.dpad_down){
                leftWinch.setTargetPosition(LIFT_BOTTOM);
                rightWinch.setTargetPosition(LIFT_BOTTOM);
                leftWinch.setPower(1);
                rightWinch.setPower(1);
            }
            if(gamepad2.left_bumper){
                leftIntake.setPosition(LCLAW_MID);
                rightIntake.setPosition(RCLAW_MID);
            }
            if(gamepad2.right_bumper){
                leftIntake.setPosition(LCLAW_CLOSE);
                rightIntake.setPosition(RCLAW_CLOSE);
            }




            //FOD ---------------------------------------------------------------------
            if (gamepad1.dpad_right)
                subtractHeading = drive.getPoseEstimate().getHeading()-Math.toRadians(270);
            if(gamepad1.dpad_up)
                subtractHeading = drive.getPoseEstimate().getHeading();
            if(gamepad1.dpad_left)
                subtractHeading = drive.getPoseEstimate().getHeading()-Math.toRadians(90);
            if(gamepad1.dpad_down||gamepad1.left_stick_button)
                subtractHeading = drive.getPoseEstimate().getHeading()-Math.toRadians(180);
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading() + subtractHeading);
            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            if(gamepad1.left_bumper){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX()/2,
                                input.getY()/2,
                                -gamepad1.right_stick_x/2
                        )
                );
            }
            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
            }
            telemetry.addData("winchCount", leftWinch.getCurrentPosition());
            telemetry.addData("armCount", chainBar.getCurrentPosition());
            telemetry.addLine("----------------------------");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
//            telemetry.addData("turretCount", turret.getCurrentPosition());
            telemetry.update();
            // Update everything. Odometry. Etc.
            drive.update();
            // Print pose to telemetry
        }
    }
}
