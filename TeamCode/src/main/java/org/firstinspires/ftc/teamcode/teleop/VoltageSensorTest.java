package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.BotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "VoltageSensorTest")
public class VoltageSensorTest extends Robot {
    private VoltageSensor voltageSensor;
    @Override
    public void runOpMode() {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        armPosition = 1;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("voltage drawn", voltageSensor.getVoltage());
            try {
                handleArm();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }
}
