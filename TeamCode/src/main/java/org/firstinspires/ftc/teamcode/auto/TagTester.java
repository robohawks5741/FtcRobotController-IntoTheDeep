package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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



//
    // UNITS ARE METERS
    double tagsize = 0.1016;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
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
        telemetry.addLine("Absolute pose: x = " + absolutePose.x() + ", y = " + absolutePose.y()
        + ", z = " + absolutePose.z() + ", angle = " + absolutePose.angle());
    }
    //this is written assuming origin is the center of the field
    //0 degrees is pointing away from audience, towards blue net zone and red ascent zone
    TagConstants.Pose pointToPose(AprilTagDetection detection) {
        AprilTagPipeline.Pose D0Fpose = AprilTagPipeline.poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);

        telemetry.addLine("RVec: " + D0Fpose.getRvec().get(0, 0)[0] + ", "+ D0Fpose.getRvec().get(1, 0)[0] + ", "+ D0Fpose.getRvec().get(2, 0)[0]);
        telemetry.addLine("TVec: " + D0Fpose.getTvec().get(0, 0)[0] * FEET_PER_METER + ", "
                + D0Fpose.getTvec().get(1, 0)[0] * FEET_PER_METER + ", "+ D0Fpose.getTvec().get(2, 0 )[0]* FEET_PER_METER);
        double xInit = detection.pose.x * FEET_PER_METER / FEET_PER_INCH;
        double yInit = detection.pose.y * FEET_PER_METER / FEET_PER_INCH;
        double yaw = D0Fpose.getRvec().get(2, 0)[0];
        double r = Math.sqrt(Math.pow(xInit, 2) + Math.pow(yInit, 2));
        double headingWithYaw = Math.atan(xInit / yInit);
        //the bot's heading relative to the wall
        double trueHeading = headingWithYaw + yaw - Math.PI;
        //reorient with tag position
        //there's very likely to be some sign error/90 degree off angle here
        trueHeading = -trueHeading + Math.toRadians(TagConstants.TagPositions.getA(detection.id));
        double trueX = -r * Math.sin(trueHeading) + TagConstants.TagPositions.getX(detection.id);
        double trueY = -r * Math.cos(trueHeading) + TagConstants.TagPositions.getY(detection.id);
        double z = -detection.pose.z + TagConstants.TagPositions.getZ(detection.id);
        return new TagConstants.Pose(trueX, trueY, z, trueHeading);
    }
}