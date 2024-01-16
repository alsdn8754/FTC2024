package org.firstinspires.ftc.teamcode.drive.FTC2024;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoCodeLib {


    DcMotor armMotor = hardwareMap.dcMotor.get("ARM");
    DcMotor grabMotor = hardwareMap.dcMotor.get("grab");
    Servo leftHandServo = hardwareMap.servo.get("leftHand");
    Servo wristServo = hardwareMap.servo.get("wrist");
    Servo rightHandServo = hardwareMap.servo.get("rightHand");

    public void A_A_W_adjust(double ArmPower, int ArmTarget, double GrabPower, int GrabTarget, double WristTarget) {


        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setTargetPosition(ArmTarget);
        grabMotor.setTargetPosition(GrabTarget);
        wristServo.setPosition(WristTarget);

        armMotor.setPower(ArmPower);
        grabMotor.setPower(GrabPower);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.update();
    }

    public void grip_adjust(double Left_Target, double Right_Target) {
        leftHandServo.setPosition(Left_Target);
        rightHandServo.setPosition(Right_Target);

        telemetry.update();
    }

    public void init_hand() {
        wristServo.setPosition(0.82);
        leftHandServo.setPosition(0.15);
        rightHandServo.setPosition(0.85);
    }


}
