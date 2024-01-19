package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class OdometryMarkerTest extends LinearOpMode {

    double rightopen = 0.5;
    double leftopen = 0.5;

    double rightclose = 0.85;
    double leftclose = 0.15;

    public void customSleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    DcMotor armMotor;
    DcMotor grabMotor;
    Servo leftHandServo;
    Servo wristServo;
    Servo rightHandServo;

    public void aawAdjust(double ArmPower, int ArmTarget, double GrabPower, int GrabTarget, double WristTarget) {

        armMotor.setTargetPosition(ArmTarget);
        grabMotor.setTargetPosition(GrabTarget);
        wristServo.setPosition(WristTarget);

        armMotor.setPower(ArmPower);
        grabMotor.setPower(GrabPower);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        grabMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("armta", ArmTarget);
        telemetry.addData("grta", GrabTarget);

        telemetry.update();
    }

    public void gripAdjust(double Left_Target, double Right_Target) {
        leftHandServo.setPosition(Left_Target);
        rightHandServo.setPosition(Right_Target);

        telemetry.update();
    }

    @Override
    public void runOpMode() {

        armMotor = hardwareMap.dcMotor.get("ARM");
        grabMotor = hardwareMap.dcMotor.get("grab");
        leftHandServo = hardwareMap.servo.get("leftHand");
        wristServo = hardwareMap.servo.get("wrist");
        rightHandServo = hardwareMap.servo.get("rightHand");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(28, -65, 0));

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(27, -65))
                .lineToLinearHeading(new Pose2d(28, -41, Math.toRadians(90)))

                .addTemporalMarker(1, () -> {
                    // Run your action in here!

                    aawAdjust(0, 0, 1, 600, 0.5);

                })

                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(60, -37, Math.toRadians(0)))

                .addTemporalMarker(1.6, () -> {
                    // Run your action in here!

                    aawAdjust(1, 400, 1, 1800, 0.67);

                })

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(60, -13, Math.toRadians(270)))

                .addTemporalMarker(1, () -> {
                    // Run your action in here!
                    // Drop servo, start motor, whatever

                    aawAdjust(1, 0, 1, 20, 0.82);

                })

                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(15)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        leftHandServo.setPosition(leftclose);
        rightHandServo.setPosition(rightclose);  //init claw close

        drive.followTrajectory(traj1);  //move to spike place, extend arm

        gripAdjust(leftopen, rightclose);  //drop purple pixel

        customSleep(100);  //wait for drop (purple)

        gripAdjust(leftclose, rightclose);  //close grip

        drive.followTrajectory(traj2);  //move to backstage, adjust angle and length

        gripAdjust(leftclose, rightopen);  //drop yellow pixel
        customSleep(100);  //wait for drop

        gripAdjust(leftclose, rightclose);  //close grip

        drive.followTrajectory(traj3);  //move to parking zone and init arm position

        drive.followTrajectory(traj4);  //move to parking zone

        customSleep(2000);


    }
}
