package org.firstinspires.ftc.teamcode.Autonom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotSmash;
import org.firstinspires.ftc.teamcode.camera.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="RoadRunnerDreapta")
public class AutoEx extends LinearOpMode
{
    int autoCase = 2;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;


    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(36, -66, Math.toRadians(90));
        Pose2d leftFrontPose = new Pose2d(36, -12, Math.toRadians(90));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotSmash robotSmash = new RobotSmash(hardwareMap);

        robotSmash.CloseClaw();
        drive.setPoseEstimate(startPose);

        //region AprilTag
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    autoCase = tag.id;
                    telemetry.addData("Id", autoCase);
                    break;
                }

            }
            sleep(20);
            telemetry.update();
        }
        //endregion


        TrajectorySequence trajMid = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(1, () -> {
                    robotSmash.setLiftTarget(200);
                })
                .forward(50)
//                .lineToLinearHeading(leftFrontPose)
                .addTemporalMarker(3, () -> {
                    robotSmash.setLiftTarget(1500);
                })
                .strafeLeft(12)
                .forward(4)
                .waitSeconds(1)
                .addTemporalMarker(0.5, () -> {
                    robotSmash.OpenClaw();
                }) //preload start
                .back(2)
//                .waitSeconds(1)
                .strafeRight(15)
                .addTemporalMarker(0.1, () -> {
                    robotSmash.setLiftTarget(500);
                })
                .waitSeconds(3)
                .turn(Math.toRadians(-90))
                .forward(26)
                .waitSeconds(1)
                .addTemporalMarker(0.5, () -> {
                    robotSmash.CloseClaw();
                }) // high to first cone

                .back(45)
                .strafeLeft(12)
//                .addDisplacementMarker(() -> {
//                     robotSmash.HighLiftPos();
//                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robotSmash.OpenClaw();
                })
                .waitSeconds(1)
//                .addDisplacementMarker(() -> {
//                    robotSmash.setLiftTarget(-5);
//                })
                .waitSeconds(1)
                .strafeRight(12)
                .forward(36)
                .build();

        drive.followTrajectorySequenceAsync(trajMid);

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            robotSmash.LiftPID();

            telemetry.update();
        }
    }



}