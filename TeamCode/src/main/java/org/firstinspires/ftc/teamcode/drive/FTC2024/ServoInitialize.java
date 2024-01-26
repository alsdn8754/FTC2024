package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
//@TeleOp

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

            leftHandServo.setPosition(0.2);//open 0.5, close 0.2
            wristServo.setPosition(0.82);//middle 0.5, high 0.82
            rightHandServo.setPosition(0.8);//open 0.5, close 0.8





        }

    }

}