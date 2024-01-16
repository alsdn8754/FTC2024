package org.firstinspires.ftc.teamcode.drive.FTC2024;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "CenterStage5048AutonomousTest", group = "Concept")
public class CenterStage5048AutonomousTest extends LinearOpMode {

    private SampleMecanumDrive drive; // 로봇의 드라이브 클래스
    private double WHEEL_RADIUS = 0.944882; // 휠의 반경 설정 (인치 단위)
    private TfodProcessor customTfod; // 커스텀 TFOD

    @Override
    public void runOpMode() {
        // 드라이브 클래스 초기화
        drive = new SampleMecanumDrive(hardwareMap);

        // 로봇의 시작 위치 설정
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        // 커스텀 TFOD 초기화
        initCustomTfod();

        // waitForStart() 메소드를 사용하여 자율주행 시작 신호를 기다립니다.
        waitForStart();

        // 예시로 12인치만큼 직진하는 동작을 수행하도록 작성하였습니다.
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(12 * WHEEL_RADIUS) // 12인치만큼 직진
                .build();

        // 이동 시작
        drive.followTrajectory(trajectory);

        // 추가적인 자율주행 동작을 수행할 수 있습니다.

        // 커스텀 TFOD 종료
        customTfod.shutdown();

        // OpMode 종료
        stop();
    }

    private void initCustomTfod() {
        // 카메라 초기화
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        VisionPortal.init(hardwareMap.appContext, webcamName, BuiltinCameraDirection.DEFAULT);

        // 커스텀 TFOD 초기화
        customTfod = new TfodProcessor(VisionPortal.getInstance().tfodMonitorViewId());
        customTfod.initialize();
        customTfod.activate();
    }

    private void processRecognitions(List<Recognition> recognitions) {
        // 카메라 화면에서 인식된 객체에 따라 다른 코드 실행
        if (recognitions != null && recognitions.size() > 0) {
            for (Recognition recognition : recognitions) {
                double objectX = recognition.getLeft(); // 객체의 x좌표
                double imageWidth = recognition.getImageWidth(); // 카메라 화면의 가로 길이

                // 카메라 화면을 가로로 3등분하여 객체의 위치에 따라 다른 코드 실행
                if (objectX < imageWidth / 3) {
                    // 첫 번째 영역에서 객체를 인식한 경우에 실행할 코드
                    // 실행할 동작을 작성하세요.
                } else if (objectX < imageWidth * 2 / 3) {
                    // 두 번째 영역에서 객체를 인식한 경우에 실행할 코드
                    // 실행할 동작을 작성하세요.
                } else {
                    // 세 번째 영역에서 객체를 인식한 경우에 실행할 코드
                    // 실행할 동작을 작성하세요.
                }
            }
        }
    }
}
