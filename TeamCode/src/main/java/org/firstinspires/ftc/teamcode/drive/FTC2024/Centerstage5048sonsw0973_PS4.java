package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class Centerstage5048sonsw0973_PS4 extends LinearOpMode {

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
        Servo shotServo = hardwareMap.servo.get("shoot");
        Servo angleServo = hardwareMap.servo.get("angle");

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


        // By setting these values to new Gamepad(), they will default to all
        // boolean values as false and all float values as 0
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // other initialization code goes here

        imu.initialize(parameters);

        waitForStart();

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isStopRequested()) return;


        int aTargetPosition = 0; //ARM
        int aCurrentPosition = 0; //adjust by code
        int gTargetPosition = 0; //grab
        int gCurrentPosition = 0;
        double wTargetPosition = 0;
        double wCurrentPosition = 0.82;
        double shooterTrigTarget = 0.02;
        double shooterAngleTarget = 0.2;

        int AngleErrorValue = 0; //adjust by player - encoder err

        int DroneShooterAngleStatus = 0;

        int Armhighposition = 0; //using with Gamepad2.y

        boolean isFirstRun = true;

        //Rumble Gamepad begin ENDGAME PERIOD
        Gamepad.RumbleEffect ENDGAME_ALERT = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 0.0, 90000) //Rumble wait for 90000ms = 90sec
                .addStep(1.0, 1.0, 1000) //ENDGAME RUMBLE for 1000ms = 1sec
                .build();


        while (opModeIsActive()) {

            if (isFirstRun) {  //ONLY ONCE RUNNING CODE

                gamepad1.runRumbleEffect(ENDGAME_ALERT);
                gamepad2.runRumbleEffect(ENDGAME_ALERT);

                wristServo.setPosition(wCurrentPosition);

                isFirstRun = false; /*!important!*/
            }

            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1/2 to be used in this loop iteration.
            // This is equivalent to doing this at the end of the previous
            // loop iteration, as it will run in the same order except for
            // the first/last iteration of the loop.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents the gamepad values from changing between being
            // used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Main teleop loop goes here


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double slow = 0.9 - (0.7 * gamepad1.right_trigger);



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
            if (currentGamepad2.x && !previousGamepad2.x) {
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

            if (currentGamepad2.b && !previousGamepad2.b) {
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
            if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
                if (aCurrentPosition > 3400 - AngleErrorValue) {
                    aTargetPosition = 3600 - AngleErrorValue;
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

            if (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button) {
                if (aCurrentPosition < 200 - AngleErrorValue) {
                    aTargetPosition = 0 - AngleErrorValue;
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
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                if (gCurrentPosition < 2100) {
                    gTargetPosition = gCurrentPosition + 300;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);
                    gCurrentPosition = gTargetPosition;
                } else {
                    gTargetPosition = 2200;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);
                    gCurrentPosition = gTargetPosition;
                }

            }

            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
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
                aTargetPosition = 0 - AngleErrorValue;
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

                //ArmHighPosition to 0
                Armhighposition = 0;
            }

            //high arm code    arm high: 3000, min: 0
            if (currentGamepad2.y && !previousGamepad2.y) {
                if (Armhighposition == 0) { //position 0 -> 1
                    //arm angle adjust
                    aTargetPosition = 3000 - AngleErrorValue;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);

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

                    //ArmHighPosition to 1
                    Armhighposition = 1;

                } else if (Armhighposition == 1) { //position 1 -> 2
                    //arm angle adjust
                    aTargetPosition = 2800 - AngleErrorValue;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);

                    //grab length adjust
                    gTargetPosition = 1200;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);

                    //wrist adjust
                    wTargetPosition = 0.4;
                    wristServo.setPosition(wTargetPosition);

                    //sync with current - target position
                    gCurrentPosition = gTargetPosition;
                    aCurrentPosition = aTargetPosition;
                    wCurrentPosition = wTargetPosition;

                    //ArmHighPosition to 2
                    Armhighposition = 2;

                } else if (Armhighposition == 2) { //position 2 -> 3
                    //arm angle adjust
                    aTargetPosition = 2400 - AngleErrorValue;
                    armMotor.setTargetPosition(aTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(0.8);

                    //grip length adjust
                    gTargetPosition = 1900;
                    grabMotor.setTargetPosition(gTargetPosition);
                    grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    grabMotor.setPower(1);

                    //wrist adjust
                    wTargetPosition = 0.55;
                    wristServo.setPosition(wTargetPosition);

                    //sync with current - target position
                    gCurrentPosition = gTargetPosition;
                    aCurrentPosition = aTargetPosition;
                    wCurrentPosition = wTargetPosition;

                    //ArmHighPosition to 3
                    Armhighposition = 3;

                } else if (Armhighposition == 3) { //position 3 -> 0
                    //arm angle adjust
                    aTargetPosition = 0 - AngleErrorValue;
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

                    //ArmHighPosition to 0
                    Armhighposition = 0;
                }
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


            //drone shooter trig position
            shotServo.setPosition(shooterTrigTarget);
            angleServo.setPosition(shooterAngleTarget);


            //drone shooter angle adjust
            if (currentGamepad1.touchpad && !previousGamepad1.touchpad) {
                if (DroneShooterAngleStatus == 0) { //DroneShooter angle adjust: low -> high
                    shooterAngleTarget = 0.35;
                    DroneShooterAngleStatus = 1;
                } else if (DroneShooterAngleStatus == 1) { //DroneShooter angle adjust: high -> low
                    shooterAngleTarget = 0.18;
                    DroneShooterAngleStatus = 0;
                }
            }

            //drone shooter trig launch
            if (gamepad1.dpad_up && gamepad1.y) {
                shooterTrigTarget = 0.35;

                shooterAngleTarget = 0.18;
                DroneShooterAngleStatus = 0;

            }

            if (gamepad1.dpad_down && gamepad1.a) {
                shooterTrigTarget = 0.02;
            }

            //adjust unexpected arm angle encoder error
             if (currentGamepad2.back && !previousGamepad2.back) {
                AngleErrorValue = AngleErrorValue + 50;
           }

            if (currentGamepad2.start && !previousGamepad2.start) {
                AngleErrorValue = AngleErrorValue - 50;
            }


            telemetry.addData("aEncoder", armMotor.getCurrentPosition()); //ARM
            telemetry.addData("code.aCurrent", aCurrentPosition);
            telemetry.addData("armTarget", aTargetPosition);
            telemetry.addData("angleErr", AngleErrorValue);
            telemetry.addData("y value", Armhighposition);
            telemetry.addLine();
            telemetry.addData("gEncoder", grabMotor.getCurrentPosition()); //grab
            telemetry.addData("code.gcurrent", gCurrentPosition);
            telemetry.addLine();
            telemetry.addData("wposition", wCurrentPosition);



            telemetry.update();


        }
    }

}