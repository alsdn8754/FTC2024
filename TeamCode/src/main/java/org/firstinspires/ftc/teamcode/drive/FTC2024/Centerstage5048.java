package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class Centerstage5048 extends LinearOpMode {

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
        Servo leftHandServo = hardwareMap.servo.get("leftHand");
        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo rightHandServo = hardwareMap.servo.get("rightHand");

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

        boolean bStatus = false;
        boolean bCurrent = false;
        boolean xStatus = false; //by code
        boolean xCurrent = false; //live gamepad
        boolean upStatus = false; //by code
        boolean upCurrent = false; //live gamepad
        boolean rightCurrent = false; //live gamepad
        boolean leftCurrent = false; //live gamepad
        boolean downStatus = false;
        boolean downCurrent = false;
        boolean yCurrent = false;
        int aTargetPosition = 0; //ARM
        int aCurrentPosition = 0;
        int gTargetPosition = 0; //grab
        int gCurrentPosition = 0;
        double wTargetPosition = 0;
        double wCurrentPosition = 0;
        boolean RightstickButton = false;
        boolean LeftstickButton = false;


        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double slow = 0.8 - (0.6 * gamepad1.right_trigger);

            bCurrent = gamepad2.b;
            xCurrent = gamepad2.x;
            yCurrent = gamepad2.y;
            downCurrent = gamepad2.dpad_down;
            upCurrent = gamepad2.dpad_up;
            leftCurrent = gamepad2.dpad_left;
            rightCurrent = gamepad2.dpad_right;

            RightstickButton = gamepad2.right_stick_button;
            LeftstickButton = gamepad2.left_stick_button;


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
            /*while (gamepad2.b) {
                aTargetPosition = aCurrentPosition + 130;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                aCurrentPosition = armMotor.getCurrentPosition();
            }*/

            /*while (gamepad2.x) {
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
            }*/

            /*while (gamepad2.a) {
                aTargetPosition = 0;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.2);
                aCurrentPosition = armMotor.getCurrentPosition();
            }*/

            /*while (gamepad2.y) {
                aTargetPosition = 800;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
                aCurrentPosition = armMotor.getCurrentPosition();
            }*/


            //eat Coding
            while (gamepad2.dpad_up) {
                gTargetPosition = 2200;
                grabMotor.setTargetPosition(gTargetPosition);
                grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabMotor.setPower(1);
                gCurrentPosition = gTargetPosition;

                wTargetPosition = 0.485;
                wristServo.setPosition(wTargetPosition);
                wCurrentPosition = wTargetPosition;
            }

            //initial position
            while (gamepad2.dpad_down) {
                //grab
                gTargetPosition = 20;
                grabMotor.setTargetPosition(gTargetPosition);
                grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabMotor.setPower(1);
                gCurrentPosition = gTargetPosition;

                wTargetPosition = 0.82;
                wristServo.setPosition(wTargetPosition);
                wCurrentPosition = wTargetPosition;
            }

            //wrist optional adjust
            if (gamepad2.x && !xCurrent == true) {
                if (wCurrentPosition <= 0.05) {
                    wTargetPosition = 0;
                    wristServo.setPosition(wTargetPosition);
                    wCurrentPosition = wTargetPosition;
                } else {
                    wTargetPosition = wCurrentPosition - 0.05;
                    wristServo.setPosition(wTargetPosition);
                    wCurrentPosition = wTargetPosition;
                }
            }

            if (gamepad2.b && !bCurrent == true) {
                if (wCurrentPosition >= 0.95) {
                    wTargetPosition = 1;
                    wristServo.setPosition(wTargetPosition);
                    wCurrentPosition = wTargetPosition;
                } else {
                    wTargetPosition = wCurrentPosition + 0.05;
                    wristServo.setPosition(wTargetPosition);
                    wCurrentPosition = wTargetPosition;
                }
            }

            //arm angle optional adjust
            if (gamepad2.right_stick_button && !RightstickButton == true) {
                if (aCurrentPosition > 4000) {
                    aTargetPosition = 4200;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    aCurrentPosition = aTargetPosition;
                } else {
                    aTargetPosition = aCurrentPosition + 200;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    aCurrentPosition = aTargetPosition;
                }
            }

            if (gamepad2.left_stick_button && !LeftstickButton == true) {
                if (aCurrentPosition < 200) {
                    aTargetPosition = 0;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    aCurrentPosition = aTargetPosition;
                } else {
                    aTargetPosition = aCurrentPosition - 200;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);
                    aCurrentPosition = aTargetPosition;
                }
            }


            //arm length optional adjust, max: 2400
            if (gamepad2.dpad_right && !rightCurrent == true) {
                if (gCurrentPosition < 2100) {
                    gTargetPosition = gCurrentPosition + 300;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);
                    gCurrentPosition = gTargetPosition;
                } else {
                    gTargetPosition = 2400;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);
                    gCurrentPosition = gTargetPosition;
                }

            }

            if (gamepad2.dpad_left && !leftCurrent == true) {
                if (gCurrentPosition > 310) {
                    gTargetPosition = gCurrentPosition - 300;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);
                    gCurrentPosition = gTargetPosition;
                } else {
                    gTargetPosition = 20;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);
                    gCurrentPosition = gTargetPosition;
                }
            }

            //init arm code    arm high: 800, min: 0
            //init wrist servo = 0.82
            while (gamepad2.a) {
                //arm angle adjust
                aTargetPosition = 0;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.8);

                //grip length adjust
                gTargetPosition = 20;
                grabMotor.setTargetPosition(gTargetPosition);
                grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabMotor.setPower(1);

                //wrist adjust
                wTargetPosition = 0.82;
                wristServo.setPosition(wTargetPosition);

                //sync with current - target position
                gCurrentPosition = gTargetPosition;
                aCurrentPosition = aTargetPosition;
                wCurrentPosition = wTargetPosition;


            }

            //high arm code    arm high: 900, min: 0
            while (gamepad2.y) {
                //arm angle adjust
                aTargetPosition = 3000;
                armMotor.setTargetPosition(aTargetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);

                //grab length adjust
                gTargetPosition = 600;
                grabMotor.setTargetPosition(gTargetPosition);
                grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                grabMotor.setPower(1);

                //wrist adjust
                wTargetPosition = 0.45;
                wristServo.setPosition(wTargetPosition);

                //sync with current - target position
                gCurrentPosition = gTargetPosition;
                aCurrentPosition = aTargetPosition;
                wCurrentPosition = wTargetPosition;
            }


            //right_grip code
            if (gamepad2.right_bumper) {
                rightHandServo.setPosition(0.5);
            } else {
                rightHandServo.setPosition(0.85);
            }

            //left_grip code
            if (gamepad2.left_bumper) {
                leftHandServo.setPosition(0.5);
            } else {
                leftHandServo.setPosition(0.15);
            }


            telemetry.addData("aEncoder", armMotor.getCurrentPosition()); //ARM
            telemetry.addData("code.acurrent", aCurrentPosition);
            telemetry.addData("bCurrent", bCurrent);
            telemetry.addData("xCurrent", xCurrent);
            telemetry.addData("gEncoder", grabMotor.getCurrentPosition()); //grab
            telemetry.addData("code.gcurrent", gCurrentPosition);
            telemetry.addData("upCurrent", upCurrent);
            telemetry.addData("upStatus", upStatus);
            telemetry.addData("wposition", wCurrentPosition);
            telemetry.addData("armtarget", aTargetPosition);


            telemetry.update();


        }
    }

}