package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Point;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.subsystems.TagConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@ Autonomous(name="Vision Tester")
public class TagTester extends LinearOpMode
{



    static Mat cameraMatrix;


    OpenCvCamera camera;
    AprilTagPipeline aprilTagDetectionPipeline;
    static double tagsizeX = 0.1016;
    static double tagsizeY = 0.1016;
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



//
    // UNITS ARE METERS
    double tagsize = 0.1016;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);



        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }





        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {

            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        //this pose is in inches
        TagConstants.Pose absolutePose = pointToPose(detection);


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(absolutePose.x(), absolutePose.y(), absolutePose.angle()));

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        telemetry.addLine("Absolute pose: x = " + absolutePose.x() + ", y = " + absolutePose.y()
        + ", z = " + absolutePose.z() + ", angle = " + Math.toDegrees(absolutePose.angle()));
    }
    //this is written assuming origin is the center of the field
    //0 degrees is pointing away from audience, towards blue net zone and red observation zone
    public static TagConstants.Pose pointToPose(AprilTagDetection detection) {
        AprilTagPipeline.Pose D0Fpose = AprilTagPipeline.poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);

        double xInit = detection.pose.x * FEET_PER_METER / FEET_PER_INCH;
        //weirdness between what's considered y and z I think
        double yInit = detection.pose.z * FEET_PER_METER / FEET_PER_INCH;
        double yaw = D0Fpose.getRvec().get(2, 0)[0];
        double r = Math.sqrt(Math.pow(xInit, 2) + Math.pow(yInit, 2));
        double headingWithYaw = Math.atan(xInit / yInit);
        //the bot's heading relative to the wall
        double trueHeading = headingWithYaw + yaw;




        //reorient with tag position
        //there's very likely to be some sign error/90 degree off angle here
        trueHeading = -trueHeading + Math.toRadians(TagConstants.TagPositions.getA(detection.id));
        //not sure about any of this to be honest I'm just trying stuff
        //this should work I think--it's disgusting but I don't feel like wading through all this trig to find
        //one sign error
        double trueX, trueY;
        if(detection.id == 12 || detection.id == 15) {
            trueX = r * Math.cos(trueHeading) + TagConstants.TagPositions.getX(detection.id);
            trueY = -r * Math.sin(trueHeading) + TagConstants.TagPositions.getY(detection.id);
        }
        else {
            trueX = -r * Math.cos(trueHeading) + TagConstants.TagPositions.getX(detection.id);
            trueY = r * Math.sin(trueHeading) + TagConstants.TagPositions.getY(detection.id);
        }
        double z = -detection.pose.y * FEET_PER_METER / FEET_PER_INCH + TagConstants.TagPositions.getZ(detection.id);
        return new TagConstants.Pose(trueX, trueY, z, trueHeading);
    }




}