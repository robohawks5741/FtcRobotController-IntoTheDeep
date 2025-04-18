package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

 /*
        Drive motors:
        leftFront
        rightFront
        leftBack
        rightBack
        Expansion hub motors:
        frontRotate
        backRotate
        liftBack
        liftFront
         */

public class Robot extends LinearOpMode {
 //   protected OpenCvCamera camera;
    protected AprilTagPipeline aprilTagDetectionPipeline;
    protected AprilTagDetection tagOfInterest = null;
    protected boolean tagFound = false;

    protected final boolean CLAW_CONTINUOUS = true;
    protected DcMotorEx frontRotate, backRotate, frontLift, backLift;
    public DualMotor rotate;
    public DualMotor lift;
    protected Servo clawIntake;
    protected Servo clawRotate;
    protected AnalogInput rotateEncoder, liftEncoder;
    protected int rotatePos;
    protected int liftPos;
    protected double rotateTargetPosition;
    protected double liftTargetVoltage;
    protected double startingRotateVoltage;
    protected double startingLiftVoltage;
    protected double liftPreviousVoltage;
    protected double desiredExtendedness;
    //this tracks voltage beyond the 0-3.2 scale and goes all the way to the LIFT_EXTENDED_VOLTS value
    protected double liftRealVoltage;
    protected DcMotorEx encoderMotor;
    protected double extendedness;
    protected int liftEncoderRotations;

    public int armPosition;
    public boolean stopArm = false;
    protected double armTicksOffset = 0;

    protected boolean hanging = false;
    protected boolean retracting = false;


    protected boolean isDown; //Checks to see if the arm is down (Parallel to the ground or lower)

    protected boolean isIn; //Checks to see if the arm is pulled in below a certain intake threshold

    protected double liftPower = 0;
    protected double rotatePower = 0;

    protected double botX;
    protected double botY;
    protected double botZ;
    protected boolean rotatePidIsReset = false;
    protected boolean extendPidIsReset = false;
    protected int debug = 0;

    protected MecanumDrive drive;

    public boolean pressed = false;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        frontRotate = hardwareMap.get(DcMotorEx.class, "frontRotate");
        backRotate = hardwareMap.get(DcMotorEx.class, "backRotate");
        frontLift = hardwareMap.get(DcMotorEx.class, "liftFront");
        backLift = hardwareMap.get(DcMotorEx.class, "liftBack");
        clawIntake = hardwareMap.get(Servo.class, "clawIntake");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


   /*     int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagPipeline();*/

        try {
            //dividing by voltspertick because all the inputs are in units of volts, so the
            //pid constants need to be in units of inverse volts rather than inverse ticks

        } catch(Exception e) {
            throw new RuntimeException(e);
        }
        encoderMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        rotateEncoder = hardwareMap.get(AnalogInput.class, "rotateEncoder");
        liftEncoder = hardwareMap.get(AnalogInput.class, "liftEncoder");
        startingRotateVoltage = normalizeRotateVoltage(rotateEncoder.getVoltage());
        startingLiftVoltage = liftEncoder.getVoltage();
        rotateTargetPosition = startingRotateVoltage;
        liftTargetVoltage = startingLiftVoltage;
        liftPreviousVoltage = startingLiftVoltage;
        //currently unused
        desiredExtendedness = 1;

        //ideally where this should go, if its able to work




        lift = new DualMotor(backLift,
                BotConstants.liftKp,
                BotConstants.liftKi,
                BotConstants.liftKd);
        try {
            rotate = new DualMotor(backRotate, frontRotate,
                    BotConstants.armUpKp,
                    BotConstants.armUpKi,
                    BotConstants.armUpKd);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }


        frontRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPos = BotConstants.LIFT_RETRACTED_TICKS;
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*Since the encoder makes multiple full revolutions in the lift extension, it's necessary to
          track how many rotations it's made so far to know the actual position for PID. This requires that
          the encoder is within the closest rotation to retracted on initialization. I don't think there's
          a great solution to this, because no matter what code you have here, the encoder can never tell
          the difference between initializing at a position and one revolution past that position
         */


     //   camera.setPipeline(aprilTagDetectionPipeline);

 /*       camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,800, OpenCvCameraRotation.UPSIDE_DOWN);
                debug = 1;
            }

            @Override
            public void onError(int errorCode)
            {

            }}

        );*/

        telemetry.setMsTransmissionInterval(50);

      /* executor = Executors.newSingleThreadExecutor(); //Camera multi-threading
        executor.submit(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                // Background processing
               // cameraUpdate();
                sleep(100);
            }
        });*/
    }

 /*   protected void cameraUpdate(){

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() > 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                tagOfInterest = tag;
                tagFound = true;

                botX = -tagOfInterest.pose.x*3.28084*12;
                botY = -tagOfInterest.pose.y*3.28084*12;
                botZ = -tagOfInterest.pose.z*3.2808*12;

                switch(tagOfInterest.id) {
                    case 11:
                        botX += TagConstants.TAG_POSITIONS.TAG11.x();
                        botY += TagConstants.TAG_POSITIONS.TAG11.y();
                        botZ += TagConstants.TAG_POSITIONS.TAG11.z();
                        break;
                    case 12:
                        botX += TagConstants.TAG_POSITIONS.TAG12.x();
                        botY += TagConstants.TAG_POSITIONS.TAG12.y();
                        botZ += TagConstants.TAG_POSITIONS.TAG12.z();
                        break;
                    case 13:
                        botX += TagConstants.TAG_POSITIONS.TAG13.x();
                        botY += TagConstants.TAG_POSITIONS.TAG13.y();
                        botZ += TagConstants.TAG_POSITIONS.TAG13.z();
                        break;
                    case 14:
                        botX += TagConstants.TAG_POSITIONS.TAG14.x();
                        botY += TagConstants.TAG_POSITIONS.TAG14.y();
                        botZ += TagConstants.TAG_POSITIONS.TAG14.z();
                        break;
                    case 15:
                        botX += TagConstants.TAG_POSITIONS.TAG15.x();
                        botY += TagConstants.TAG_POSITIONS.TAG15.y();
                        botZ += TagConstants.TAG_POSITIONS.TAG15.z();
                        break;
                    case 16:
                        botX += TagConstants.TAG_POSITIONS.TAG16.x();
                        botY += TagConstants.TAG_POSITIONS.TAG16.y();
                        botZ += TagConstants.TAG_POSITIONS.TAG16.z();
                        break;


                        }
                        drive = new MecanumDrive(hardwareMap, new Pose2d(botX, botY, 0));
                break;

            }
        } else {
            tagFound = false;
        }
    }*/

    public void handleArm() throws Exception {

        rotateTargetPosition = normalizeRotateVoltage(rotateTargetPosition);
        extendedness = (liftRealVoltage - BotConstants.LIFT_RETRACTED_VOLTS)
                / (BotConstants.LIFT_EXTENDED_VOLTS - BotConstants.LIFT_RETRACTED_VOLTS);
        //solves for target positions in ticks for rotate and lift based on the voltage values
        //these are useless i believe
        rotatePos = (int)((rotateTargetPosition - startingRotateVoltage) / BotConstants.VOLTS_PER_TICK);
        //this sign needs to be checked
        liftPos = (int)((liftTargetVoltage - startingLiftVoltage) / BotConstants.VOLTS_PER_TICK);
        //checks whether the lift encoder voltage has ticked over one way or the other
        checkLiftEncoder();

        //Checks to see if the arm is pulled in
        isIn = liftRealVoltage < BotConstants.LIFT_ROTATABLE_VOLTS;

        //Checks to see if the arm is down or up
        isDown = !(normalizeRotateVoltage(rotateEncoder.getVoltage()) < BotConstants.ARM_UP_EXTENDABLE_VOLTS);

        if(armPosition == -5) {
            liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
            rotateTargetPosition = BotConstants.ARM_STARTING_VOLTS;
        }
        else if ((isDown && armPosition > 0 && !isIn) || (!isDown && armPosition < 0 && !isIn)){ //Sees if it needs to pull in
            liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
        } else if((!isDown && armPosition < 0 && isIn) || (isDown && armPosition > 0 && isIn) || (isDown && armPosition < 3) || (!isDown && armPosition > 2)){
            //Waits until it is pulled in yo rotate it
            //is up, is going down, and is in
            //is down, is going up and is in
            //is down, is staying down
            //is up and is staying up
            //Only activates when it is down and the target is the specimen placement, high bucket, low bucket, and hang position
            if(!rotatePidIsReset) {
                rotate.resetPID();
                rotatePidIsReset = true;
            }
            switch(armPosition) {
                case -1:
                    rotateTargetPosition = BotConstants.ARM_GROUND_VOLTS_EXTENDED;
                    break;
                case -2:
                    rotateTargetPosition = BotConstants.HORIZONTAL_VOLTS;
                    break;
                case -3:
                    rotateTargetPosition = BotConstants.ROTATE_SHORT_DOWN_VOLTS;
                    break;
                case -4:
                    rotateTargetPosition = BotConstants.ROTATE_SPECIMEN_PLACEMENT;
                    break;
                case 3:
                    rotateTargetPosition = BotConstants.ARM_SPECIMEN_PLACEMENT_VOLTS;
                    break;
                case 4:
                case 5:
                    rotateTargetPosition = BotConstants.ARM_FRONT_PLACING_VOLTS;
                    break;
                case 6:
                    rotateTargetPosition = BotConstants.ARM_BACK_VOLTS;
                    break;
                case 7:
                    //this should be irrelevant
                    break;
                case 8:
                    rotateTargetPosition = BotConstants.ARM_VERTICAL_VOLTS;
                    break;
                default:
                    telemetry.addLine("Invalid arm position");
            }
            if((armPosition < 0 && isDown) || (armPosition > 0) && !isDown){

                //Is going down and is down
                //Is going up and is up
                if(!extendPidIsReset) {
                    lift.resetPID();
                    extendPidIsReset = true;
                }
                switch(armPosition) {
                    case -1:
                        liftTargetVoltage = BotConstants.LIFT_DOWN_EXTENDED_VOLTS;
                        break;
                    case -2:
                    case 8:
                    case 6:
                        liftTargetVoltage = BotConstants.LIFT_RETRACTED_SIDEWAYS_VOLTS;
                        break;
                    case -3:
                        liftTargetVoltage = BotConstants.LIFT_SHORT_DOWN_VOLTS;
                        break;
                    case -4:
                    case 3:
                        liftTargetVoltage = BotConstants.LIFT_SPECIMEN_PLACEMENT_POSITION;
                        break;
                    case 4:
                        liftTargetVoltage = BotConstants.LIFT_LOW_BUCKET;
                        break;
                    case 5:
                        liftTargetVoltage = BotConstants.LIFT_EXTENDED_VOLTS;
                        break;
                    case 7:
                        liftTargetVoltage = BotConstants.LIFT_RETRACTED_HANGING_VOLTS;
                        break;
                    default:
                        telemetry.addLine("Invalid arm position");
                }
            }

        }
        if(!retracting) {
            updateLift();
        }

        //Update arm and rotate
        if (!hanging){
            updateRotate();
        }
    }

    protected void updateRotate() throws Exception {
        calculateLiftVoltage();
        double pidPower = (rotate.getPIDPower(rotateEncoder.getVoltage(), rotateTargetPosition));
        double gravity = BotConstants.armBasePower * Math.cos(getAngle());
        double totalPower = pidPower;
        if(armPosition == 6) {
            totalPower *= 1.2;
        }
        totalPower = clamp(totalPower, -1, 1);
     //   double powerScale = .5 + 0.5*extendedness;
        rotatePower = totalPower + gravity;
        rotate.setPower(rotatePower);
       /* rotate.setTargetPosition(rotatePos);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            //signs of each part of this are based on direction of motor and encoder
            //currently up from ground = positive power, negative encoder voltage
            rotatePower = (-rotate.getPIDPower(rotateTargetVoltage, normalizeRotateVoltage(rotateEncoder.getVoltage()))
                    + BotConstants.armBasePower * Math.cos(getAngle()));
            double powerScaleFactor = (1 + extendedness) / 2;
            rotate.setPower(powerScaleFactor * clamp(rotatePower, -1, 1));
        } catch(Exception e) {
            throw new RuntimeException(e);
        }*/
    }



    protected void updateLift() {
        lift.setTargetPosition(liftPos);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        try {
            liftPower=0;
            liftPower = (lift.getPIDPower(liftRealVoltage, liftTargetVoltage));
                    //+ BotConstants.liftBasePower * Math.sin(getAngle()));
            lift.setPower(clamp(liftPower, -1, 1));
        } catch(Exception e) {
            throw new RuntimeException(e);
        }
    }
    //makes sure voltage is a single continuous scale rather than having a jump from 0 to 3.2
    protected double normalizeRotateVoltage(double v) {
        if(v > mod(BotConstants.ARM_VERTICAL_VOLTS - 0.2, BotConstants.MAX_VOLTAGE)) {
            v -= BotConstants.MAX_VOLTAGE;
        }
        return v;
    }
    protected static double clamp(double input, double min, double max){
        if(input > max) {
            return max;
        }
        if(input < min) {
            return min;
        }
        return input;
    }

    protected double getAngle() {
        return -(rotateEncoder.getVoltage() - BotConstants.HORIZONTAL_VOLTS) * BotConstants.RADS_PER_VOLT;
    }
    //TODO: figure out which direction is which for these--runClaw is defined to be in intake direction
    public void runClaw() {
        clawIntake.setPosition(1);
    }
    public void runClawReverse() {
        clawIntake.setPosition(0);
    }
    public void stopClaw() {
        clawIntake.setPosition(0.5);
    }

    //these constants need to be redetermined
    protected void rotateClawUp() {
        clawRotate.setPosition(BotConstants.SERVO_TEST_POS);
    }

    protected void rotateClawDown() {
        clawRotate.setPosition(0);
    }

    public void resetPosition() {
        resetPid();
        rotatePidIsReset = false;
        extendPidIsReset = false;
    }

    public void resetPid() {
        lift.resetPID();
        rotate.resetPID();
    }

    protected void calculateLiftVoltage() {
        liftRealVoltage = startingLiftVoltage + (-encoderMotor.getCurrentPosition() - armTicksOffset) * BotConstants.INCREMENTAL_TO_VOLTS;
        liftRealVoltage += normalizeRotateVoltage(rotateEncoder.getVoltage()) -
                normalizeRotateVoltage(BotConstants.ARM_VERTICAL_VOLTS);
    }

    protected void checkLiftEncoder() {
        //this is one idea for doing this, might need to be tweaked/reworked depending on how jumpy the encoder is
        if(liftEncoder.getVoltage() < 1.5 && liftPreviousVoltage > BotConstants.MAX_VOLTAGE - 1.5) {
            liftEncoderRotations++;
        }
        else if(liftEncoder.getVoltage() > liftEncoder.getMaxVoltage() - 0.5 && liftPreviousVoltage < 0.5) {
            liftEncoderRotations--;
        }
        //Which of these if statements is incrementing/decrementing depends on the direction positive voltage
        //is on the extension, which needs to be checked
        liftPreviousVoltage = liftEncoder.getVoltage();
    }

    protected void openClaw() {
        clawIntake.setPosition(BotConstants.CLAW_OPEN);
    }
    protected void closeClaw() {
        clawIntake.setPosition(BotConstants.CLAW_CLOSED);
    }
    //TODO: create code for moving claw along ground

    //mod that works for doubles
    protected static double mod(double a, double b) {
        while(a < 0) {
            a += b;
        }
        while(a > b) {
            a -= b;
        }
        return a;
    }
}