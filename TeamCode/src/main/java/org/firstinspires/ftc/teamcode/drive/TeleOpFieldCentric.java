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
public class TeleOpFieldCentric extends LinearOpMode {

    DcMotor leftWinch;
    DcMotor rightWinch;
    DcMotor turret;
    DcMotor chainBar;
    Servo leftIntake;
    Servo rightIntake;

    double subtractHeading = 0;
    //final double CHAIN_BAR_DELAY_TIME = 0.0;
    //private ElapsedTime liftTimer = new ElapsedTime();

    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_RETRACT
    }

    public enum ArmState {
        ARM_UPPER,
        ARM_LOWER,
        ARM_MIDDLE,
        ARM_REVERSE
    }

    LiftState liftState = LiftState.LIFT_START;
    ArmState armState = ArmState.ARM_LOWER;

    final int LIFT_LOW = 0;
    final int LIFT_LOW2 = 1200;
    final int LIFT_MID = 3100;
    final int LIFT_HIGH = 4050;
    final int ARM_LOW = 0;
    final int ARM_MID = 40;
    final int ARM_HIGH = 80;
    final int ARM_BACK = -200;
    int liftGoal = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        turret = hardwareMap.dcMotor.get("turret");
        chainBar = hardwareMap.dcMotor.get("chainBar");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        leftIntake.setPosition(.75);
        rightIntake.setPosition(0.05);

        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightWinch.setTargetPosition(LIFT_LOW);
        leftWinch.setTargetPosition(LIFT_LOW);
//        chainBar.setTargetPosition(ARM_LOW);
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
            switch (liftState) {
                case LIFT_START:
//                    leftWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    rightWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    leftWinch.setPower(.05);
//                    rightWinch.setPower(.05);
                    if (gamepad2.y) {
//                        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftWinch.setTargetPosition(LIFT_HIGH);
                        rightWinch.setTargetPosition(LIFT_HIGH);
//                        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftWinch.setPower(.75);
                        rightWinch.setPower(.75);
                        liftGoal = LIFT_HIGH;
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    else if(gamepad2.a){
//                        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftWinch.setTargetPosition(LIFT_LOW2);
                        rightWinch.setTargetPosition(LIFT_LOW2);
//                        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftWinch.setPower(.75);
                        rightWinch.setPower(.75);
                        liftGoal = LIFT_LOW2;
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    else if(gamepad2.b){
//                        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        leftWinch.setTargetPosition(LIFT_MID);
                        rightWinch.setTargetPosition(LIFT_MID);
//                        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftWinch.setPower(.75);
                        rightWinch.setPower(.75);
                        liftGoal = LIFT_MID;
                        liftState = LiftState.LIFT_EXTEND;
                    }

                    break;
                case LIFT_EXTEND:
                    if (Math.abs(leftWinch.getCurrentPosition()-liftGoal)<5
                            || Math.abs(rightWinch.getCurrentPosition()-liftGoal)<5){
                        liftGoal = LIFT_LOW;
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if(gamepad2.dpad_down){
                        leftWinch.setTargetPosition(LIFT_LOW);
                        rightWinch.setTargetPosition(LIFT_LOW);
                        leftWinch.setPower(-.75);
                        rightWinch.setPower(.75);
                    }
                    if(Math.abs(leftWinch.getCurrentPosition()-liftGoal)<5 || Math.abs(rightWinch.getCurrentPosition()-liftGoal)<5)
                        liftState = LiftState.LIFT_START;
                    break;
                default:
                    liftState = LiftState.LIFT_START;
            }
//            switch(armState){
//                case ARM_LOWER:
//                    if(gamepad1.dpad_up){
//                        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        chainBar.setTargetPosition(ARM_HIGH);
//                        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        chainBar.setPower(1);
//                    }
//                    armState = ArmState.ARM_UPPER;
//                    break;
//                case ARM_UPPER:
//                    if(gamepad1.dpad_down){
//                        chainBar.setTargetPosition(ARM_LOW);
//                        chainBar.setPower(1);
//                        if(Math.abs(chainBar.getCurrentPosition()-ARM_LOW)<5){
//                            chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                            chainBar.setPower(0);
//                            armState = ArmState.ARM_LOWER;
//                        }
//                    }
//                    break;
//            }
            // rotate turret
            if(gamepad2.dpad_left)
                turret.setPower(.6);
            else if(gamepad2.dpad_right)
                turret.setPower(-.6);
            else turret.setPower(0);

            //intake
            if(gamepad2.right_bumper) {
                leftIntake.setPosition(.45);
                rightIntake.setPosition(.35);
            }
            else if(gamepad2.left_bumper) {
                leftIntake.setPosition(.75);
                rightIntake.setPosition(0.05);
            }

            //chain bar
//            if(gamepad1.dpad_up){
//                chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                chainBar.setTargetPosition(ARM_HIGH);
//                chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                chainBar.setPower(1);
//            }
//            if(gamepad1.dpad_right){
//                chainBar.setTargetPosition(ARM_MID);
//                chainBar.setPower(1);
//            }
//            if(gamepad1.dpad_down){
//                chainBar.setTargetPosition(ARM_LOW);
//                chainBar.setPower(1);
//                if(Math.abs(chainBar.getCurrentPosition()-ARM_LOW)<5){
//                    chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    chainBar.setPower(0);
//                }
//            }
            chainBar.setPower(gamepad1.right_trigger);
//            chainBar.setPower(-gamepad1.left_trigger);
            //reset FOD
            if (gamepad1.right_stick_button)
                subtractHeading = drive.getPoseEstimate().getHeading();

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
                                -gamepad1.right_stick_x
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
            telemetry.addData("state", liftState);
            telemetry.addData("winchCount", leftWinch.getCurrentPosition());
            telemetry.addData("armCount", chainBar.getCurrentPosition());
            telemetry.update();
            // Update everything. Odometry. Etc.
            drive.update();
            // Print pose to telemetry
        }
    }
}
