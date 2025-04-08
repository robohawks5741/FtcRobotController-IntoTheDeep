package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.subsystems.TagConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoTesting extends AutoSuper {

    Mat cameraMatrix;


    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;
    double tagsizeX = 0.1016;
    double tagsizeY = 0.1016;
    static final double FEET_PER_METER = 3.28084;
    static final double FEET_PER_INCH = 1.0/12;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 909.781;
    double fy = 909.781;
    double cx = 627.946;
    double cy = 354.307;
    double tagsize = 0.1016;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean pressed = false;
        cameraMatrix = new Mat(3,3,CvType.CV_32FC1);
        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);
        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,800, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a && !pressed) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                if(!currentDetections.isEmpty()) {
                    pressed = true;
                    tagOfInterest = currentDetections.get(0);
                    TagConstants.Pose currentPose = TagTester.pointToPose(tagOfInterest);
                    beginPose = new Pose2d(currentPose.x(), currentPose.y(), currentPose.angle());
                    Actions.runBlocking(new ParallelAction(
                            drive.actionBuilder(beginPose)
                                    .splineToConstantHeading(new Vector2d(-50, 0), 0)
                                    .build()));
                }
            }
        }
    }
}