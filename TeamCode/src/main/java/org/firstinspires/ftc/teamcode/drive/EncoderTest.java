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
public class EncoderTest extends LinearOpMode {

    DcMotor leftWinch;
    DcMotor rightWinch;
    //DcMotor turret;
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
    final int LIFT_HIGH = 4600;
    final int ARM_LOW = 0;
    final int ARM_MID = 40;
    final int ARM_HIGH = 80;
    final int ARM_BACK = -200;
    int liftGoal = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftWinch = hardwareMap.dcMotor.get("leftWinch");
        rightWinch = hardwareMap.dcMotor.get("rightWinch");
        //turret = hardwareMap.dcMotor.get("turret");
        chainBar = hardwareMap.dcMotor.get("chainBar");
        leftIntake = hardwareMap.servo.get("leftIntake");
        rightIntake = hardwareMap.servo.get("rightIntake");
        leftIntake.setPosition(.64);
        rightIntake.setPosition(0.16);

//        leftWinch.setDirection(DcMotor.Direction.REVERSE);
        chainBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        chainBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightWinch.setTargetPosition(LIFT_LOW);
//        leftWinch.setTargetPosition(LIFT_LOW);
//        chainBar.setTargetPosition(ARM_LOW);
//        rightWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        chainBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        chainBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            chainBar.setPower(gamepad1.right_trigger);
            chainBar.setPower(-gamepad1.left_trigger);

            telemetry.addData("leftCount", leftWinch.getCurrentPosition());
            telemetry.addData("rightCount", rightWinch.getCurrentPosition());
            telemetry.addData("armCount", chainBar.getCurrentPosition());
            telemetry.update();
            // Update everything. Odometry. Etc.
            drive.update();
            // Print pose to telemetry
        }
    }
}
