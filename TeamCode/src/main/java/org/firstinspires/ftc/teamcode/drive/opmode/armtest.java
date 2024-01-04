package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp

public class armtest extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {

// Declare our motors
// Make sure your ID's match your configuration

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        DcMotor armMotor = hardwareMap.dcMotor.get("ARM");

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

        if (isStopRequested()) return;

        boolean aStatus = false;
        boolean aCurrent = false;
        boolean upStatus = false;
        boolean downStatus = false;
        int targetPosition = 0;
        int currentPosition = 0;
        int gPosition = 0;
        int wPosition = 0;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double ay = -gamepad2.left_stick_y;
            float slow = (float) (gamepad1.right_trigger + 1.5);
            aCurrent = gamepad2.a;
            boolean downCurrent = gamepad2.dpad_left;
            boolean upCurrent = gamepad2.dpad_right;
            armMotor.setPower(gamepad2.left_stick_y * 0.1);

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
            double frontLeftPower = ((rotY + rotX - rx) / denominator)/slow;
            double backLeftPower = ((rotY - rotX - rx) / denominator)/slow;
            double frontRightPower = ((rotY - rotX + rx) / denominator)/slow;
            double backRightPower = ((rotY + rotX + rx) / denominator)/slow;
            double armPower = ay;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            armMotor.setPower(armPower);

            while (gamepad2.b) {
                targetPosition = currentPosition + 130;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                currentPosition = armMotor.getCurrentPosition();
            }
                while (gamepad2.x) {
                    if (targetPosition > 10) {
                        targetPosition = currentPosition - 130;
                        armMotor.setTargetPosition(targetPosition);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(0.2); currentPosition = armMotor.getCurrentPosition();
                    }
                    else {
                        targetPosition = 10; armMotor.setTargetPosition(targetPosition);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        armMotor.setPower(0.2);
                        currentPosition = armMotor.getCurrentPosition();
                    }
                }

            while (gamepad2.a) {
                targetPosition = 30;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                currentPosition = armMotor.getCurrentPosition();
            }

            while (gamepad2.y) {
                targetPosition = 900;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                currentPosition = armMotor.getCurrentPosition();
            }

            telemetry.addData("encoder", armMotor.getCurrentPosition());
            telemetry.addData("aCurrent", aCurrent);
            telemetry.addData("aStatus", aStatus);
            telemetry.addData("wPosition", wPosition);
            telemetry.update();



        }

    }

}