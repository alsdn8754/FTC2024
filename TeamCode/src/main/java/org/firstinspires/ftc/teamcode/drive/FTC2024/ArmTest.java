package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp

public class ArmTest extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {

// Declare our motors
// Make sure your ID's match your configuration

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        DcMotor armMotor = hardwareMap.dcMotor.get("ARM");
        DcMotor grabMotor = hardwareMap.dcMotor.get("grab");

// Reverse the right side motors. This may be wrong for your setup.
// If your robot moves backwards when commanded to go forwards,
// reverse the left side instead.
// See the note about this earlier on this page.

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

// Retrieve the IMU from the hardware map

        IMU imu = hardwareMap.get(IMU.class, "imu");

// Adjust the orientation parameters to match your robot

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

// Without this, the REV Hub's orientation is assumed to be logo up / USB forward

        imu.initialize(parameters);

        waitForStart();

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;

        boolean aStatus = false;
        boolean aCurrent = false;
        boolean upStatus = false;
        boolean upCurrent = false;
        boolean downStatus = false;
        boolean downCurrent = false;
        int aTargetPosition = 0; //ARM
        int aCurrentPosition = 0;
        int gTargetPosition = 0; //grab
        int gCurrentPosition = 0;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ay = -gamepad2.left_stick_y;
            double slow = 0.8 - (0.6 * gamepad1.right_trigger);
            aCurrent = gamepad2.a;
            downCurrent = gamepad2.dpad_left;
            upCurrent = gamepad2.dpad_right;


// This button choice was made so that it is hard to hit on accident,
// it can be freely changed based on preference.
// The equivalent button is start on Xbox-style controllers.

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

// Rotate the movement direction counter to the bot's rotation

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

// Denominator is the largest motor power (absolute value) or 1
// This ensures all the powers maintain the same ratio,
// but only if at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = ((rotY + rotX - rx) / denominator) * slow;
            double backLeftPower = ((rotY - rotX - rx) / denominator) * slow;
            double frontRightPower = ((rotY - rotX + rx) / denominator) * slow;
            double backRightPower = ((rotY + rotX + rx) / denominator) * slow;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //ARM Coding
            while (gamepad2.b) {
                aTargetPosition = aCurrentPosition + 130;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                aCurrentPosition = armMotor.getCurrentPosition();
            }
            while (gamepad2.x) {
                if (aTargetPosition > 10) {
                    aTargetPosition = aCurrentPosition - 130;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.2); aCurrentPosition = armMotor.getCurrentPosition();
                }
                else {
                    aTargetPosition = 10; armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.2);
                    aCurrentPosition = armMotor.getCurrentPosition();
                }
            }

            while (gamepad2.a) {
                aTargetPosition = 0;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                aCurrentPosition = armMotor.getCurrentPosition();
            }

            while (gamepad2.y) {
                aTargetPosition = 800;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
                aCurrentPosition = armMotor.getCurrentPosition();
            }
            //grab Coding
            while (gamepad2.dpad_up) {
                gTargetPosition = 2400;
                grabMotor.setTargetPosition(gTargetPosition);
                grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabMotor.setPower(1);
                gCurrentPosition = grabMotor.getCurrentPosition();
            }

            while (gamepad2.dpad_down) {
                gTargetPosition = 0;
                grabMotor.setTargetPosition(gTargetPosition);
                grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabMotor.setPower(1);
                gCurrentPosition = grabMotor.getCurrentPosition();
            }

            telemetry.addData("aEncoder", armMotor.getCurrentPosition()); //ARM
            telemetry.addData("aCurrent", aCurrent);
            telemetry.addData("aStatus", aStatus);
            telemetry.addData("gEncoder", grabMotor.getCurrentPosition()); //grab
            telemetry.addData("upCurrent", upCurrent);
            telemetry.addData("upStatus", upStatus);

            telemetry.update();



        }

    }

}