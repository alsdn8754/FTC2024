package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


import java.util.List;

@Autonomous(name = "RedClose", group = "Concept")
public class TensorFlowTestRedAutonomous extends LinearOpMode {


    int biconPosition = 0;
    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_ASSET = "5048Red.tflite";

    private static final String[] LABELS = {
            "RED",
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    double rightopen = 0.5;
    double leftopen = 0.5;

    double rightclose = 0.85;
    double leftclose = 0.15;

    DcMotor armMotor;
    DcMotor grabMotor;
    Servo leftHandServo;
    Servo wristServo;
    Servo rightHandServo;

    public void customSleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

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

        Trajectory trajM1 = drive.trajectoryBuilder(new Pose2d(27, -65))
                .lineToLinearHeading(new Pose2d(28, -41, Math.toRadians(90)))

                .addTemporalMarker(1, () -> {
                    // Run your action in here!

                    aawAdjust(0, 0, 1, 600, 0.5);

                })

                .build();

        Trajectory trajM2 = drive.trajectoryBuilder(trajM1.end())
                .lineToLinearHeading(new Pose2d(60, -37, Math.toRadians(0)))

                .addTemporalMarker(1.6, () -> {
                    // Run your action in here!

                    aawAdjust(1, 400, 1, 1800, 0.67);

                })

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(trajM2.end())  //RED parking
                .lineToLinearHeading(new Pose2d(60, -13, Math.toRadians(270)))

                .addTemporalMarker(1, () -> {

                    aawAdjust(1, 0, 1, 20, 0.82);

                })

                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())  //RED parking
                .strafeLeft(17)
                .build();

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.addData("position", biconPosition);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);

            while (isStopRequested()) {
                if (biconPosition == 1) {

                }
                else if (biconPosition == 2) {

                    leftHandServo.setPosition(leftclose);
                    rightHandServo.setPosition(rightclose);  //init claw close

                    drive.followTrajectory(trajM1);  //move to spike place, extend arm

                    gripAdjust(leftopen, rightclose);  //drop purple pixel

                    customSleep(100);  //wait for drop (purple)

                    gripAdjust(leftclose, rightclose);  //close grip

                    drive.followTrajectory(trajM2);  //move to backstage, adjust angle and length

                    gripAdjust(leftclose, rightopen);  //drop yellow pixel
                    customSleep(100);  //wait for drop

                    gripAdjust(leftclose, rightclose);  //close grip

                    drive.followTrajectory(traj3);  //move to parking zone and init arm position

                    drive.followTrajectory(traj4);  //move to parking zone

                    customSleep(2000);

                }
                else {

                }
            }
        }

        visionPortal.close();

    } //run opmode end

    private void initTfod() {

        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }



        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("biconPosition", biconPosition);

            if (x < 200) {
                biconPosition = 1;
            } else if (x >= 200 && x < 400) {
                biconPosition = 2;
            } else {
                biconPosition = 3;
            }


        }


    }

}
