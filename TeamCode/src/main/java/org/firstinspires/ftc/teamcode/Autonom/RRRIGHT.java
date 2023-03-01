package org.firstinspires.ftc.teamcode.Autonom;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class RRRIGHT extends LinearOpMode
{
    int autoCase = 2;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;


    int cone_5 = 225;
    int cone_4 = 165;
    int cone_3 = 115;
    int cone_2 = 70;
    int cone_1 = 20;

    int ground = 20;
    int low = 600;
    int medium  = 980;
    int high = 1380;

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

        Pose2d startPose = new Pose2d(33.00, -64.00, Math.toRadians(90));

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



        TrajectorySequence parcare1 = drive.trajectorySequenceBuilder(startPose)
                //preload
                .addTemporalMarker(() -> robotSmash.setLiftTarget(40))
                .lineToConstantHeading(new Vector2d(36.39, -45.55))
                .lineToConstantHeading(new Vector2d(35.82, -10.65))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1520))
                .lineToConstantHeading(new Vector2d(22.5, -10.51))
                .lineToConstantHeading(new Vector2d(22.5, -6.5))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())

                // first_and_second_cycle
                .lineToConstantHeading(new Vector2d(24.00, -12.10))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(225))
                .lineToLinearHeading(new Pose2d(36.97, -12.08, Math.toRadians(-0.80)))
                .lineToLinearHeading(new Pose2d(64.00, -12.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(64, -12.10))
                .addTemporalMarker(() -> robotSmash.CloseClaw())
                .waitSeconds(0.0001)
                .addTemporalMarker(() -> robotSmash.setLiftTarget(600))


                .lineToConstantHeading(new Vector2d(36.97, -12.08))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1450))
                .lineToLinearHeading(new Pose2d(24.38, -14.50, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(22.5, -6))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())


                //second cone

                .lineToConstantHeading(new Vector2d(24.00, -12.10))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(165))
                .lineToLinearHeading(new Pose2d(36.97, -12.08, Math.toRadians(-0.80)))
                .lineToLinearHeading(new Pose2d(64.00, -12.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(64, -12.10))
                .addTemporalMarker(() -> robotSmash.CloseClaw())
                .waitSeconds(0.0001)
                .addTemporalMarker(() -> robotSmash.setLiftTarget(600))


                .lineToConstantHeading(new Vector2d(36.97, -12.08))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1450))
                .lineToLinearHeading(new Pose2d(24.38, -14.50, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(22.5, -5))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())

                //park1
                .lineToSplineHeading(new Pose2d(12, -19.81, Math.toRadians(90.00)))

                .build();


        TrajectorySequence parcare2 = drive.trajectorySequenceBuilder(startPose)
                //preload
                .addTemporalMarker(() -> robotSmash.setLiftTarget(40))
                .lineToConstantHeading(new Vector2d(36.39, -45.55))
                .lineToConstantHeading(new Vector2d(35.82, -10.65))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1520))
                .lineToConstantHeading(new Vector2d(22.5, -10.51))
                .lineToConstantHeading(new Vector2d(22.5, -6.5))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())

                // first_cone

                .lineToConstantHeading(new Vector2d(24.00, -12.10))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(225))
                .lineToLinearHeading(new Pose2d(36.97, -12.08, Math.toRadians(-0.80)))
                .lineToLinearHeading(new Pose2d(64.00, -12.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(64, -12.10))
                .addTemporalMarker(() -> robotSmash.CloseClaw())
                .waitSeconds(0.0001)
                .addTemporalMarker(() -> robotSmash.setLiftTarget(600))


                .lineToConstantHeading(new Vector2d(36.97, -12.08))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1450))
                .lineToLinearHeading(new Pose2d(24.38, -14.50, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(22.5, -6))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())


                //second cone

                .lineToConstantHeading(new Vector2d(24.00, -12.10))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(165))
                .lineToLinearHeading(new Pose2d(36.97, -12.08, Math.toRadians(-0.80)))
                .lineToLinearHeading(new Pose2d(64.00, -12.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(64, -12.10))
                .addTemporalMarker(() -> robotSmash.CloseClaw())
                .waitSeconds(0.0001)
                .addTemporalMarker(() -> robotSmash.setLiftTarget(600))


                .lineToConstantHeading(new Vector2d(36.97, -12.08))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1450))
                .lineToLinearHeading(new Pose2d(24.38, -14.50, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(22.5, -5))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())

                //park2
                .lineToSplineHeading(new Pose2d(36, -19.2, Math.toRadians(90.00)))

                .build();


        TrajectorySequence parcare3 = drive.trajectorySequenceBuilder(startPose)
                //preload
                .addTemporalMarker(() -> robotSmash.setLiftTarget(40))
                .lineToConstantHeading(new Vector2d(36.39, -45.55))
                .lineToConstantHeading(new Vector2d(35.82, -10.65))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1520))
                .lineToConstantHeading(new Vector2d(22.5, -10.51))
                .lineToConstantHeading(new Vector2d(22.5, -6.5))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())

                // first_cone

                .lineToConstantHeading(new Vector2d(24.00, -12.10))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(225))
                .lineToLinearHeading(new Pose2d(36.97, -12.08, Math.toRadians(-0.80)))
                .lineToLinearHeading(new Pose2d(64.00, -12.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(64, -12.10))
                .addTemporalMarker(() -> robotSmash.CloseClaw())
                .waitSeconds(0.0001)
                .addTemporalMarker(() -> robotSmash.setLiftTarget(600))


                .lineToConstantHeading(new Vector2d(36.97, -12.08))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1450))
                .lineToLinearHeading(new Pose2d(24.38, -14.50, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(22.5, -6))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())


                //second cone

                .lineToConstantHeading(new Vector2d(24.00, -12.10))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(165))
                .lineToLinearHeading(new Pose2d(36.97, -12.08, Math.toRadians(-0.80)))
                .lineToLinearHeading(new Pose2d(64.00, -12.00, Math.toRadians(0.00)))
                .lineToConstantHeading(new Vector2d(64, -12.10))
                .addTemporalMarker(() -> robotSmash.CloseClaw())
                .waitSeconds(0.0001)
                .addTemporalMarker(() -> robotSmash.setLiftTarget(600))


                .lineToConstantHeading(new Vector2d(36.97, -12.08))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1450))
                .lineToLinearHeading(new Pose2d(24.38, -14.50, Math.toRadians(90.00)))
                .lineToConstantHeading(new Vector2d(22.5, -5))
                .addTemporalMarker(() -> robotSmash.setLiftTarget(1300))
                .waitSeconds(0.001)
                .addTemporalMarker(() -> robotSmash.OpenClaw())

                //park3
                .lineToLinearHeading(new Pose2d(36.25, -13.08, Math.toRadians(120.00)))
                .lineToLinearHeading(new Pose2d(56.5, -14.37, Math.toRadians(90.00)))
                .setReversed(true)
                .build();



        if(autoCase==1)
        drive.followTrajectorySequenceAsync(parcare1);
        else if(autoCase==2)
        drive.followTrajectorySequenceAsync(parcare2);
        else if(autoCase==3)
        drive.followTrajectorySequenceAsync(parcare3);



        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            robotSmash.LiftPID();

            telemetry.update();
        }
    }



}
