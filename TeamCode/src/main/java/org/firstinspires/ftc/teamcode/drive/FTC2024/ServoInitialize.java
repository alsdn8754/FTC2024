package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class ServoInitialize extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {

// Declare our motors
// Make sure your ID's match your configuration

        Servo leftHandServo = hardwareMap.servo.get("leftHand");
        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo rightHandServo = hardwareMap.servo.get("rightHand");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            leftHandServo.setPosition(0.5);
            wristServo.setPosition(0.5);
            rightHandServo.setPosition(0.5);



        }

    }

}