package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class OdometryTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(27,-63))
                .splineToConstantHeading(new Vector2d(18, 18), Math.toRadians(0))
                .build();

       /* Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineToConstantHeading(new Vector2d(18, 18), Math.toRadians(0))
                .build();*/

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        //drive.followTrajectory(traj2);

    }
}