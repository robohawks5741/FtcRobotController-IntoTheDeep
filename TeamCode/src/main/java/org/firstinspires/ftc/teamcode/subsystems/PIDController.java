package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp, Kd, Ki;
    private double integralSum = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();
    public PIDController(double K_P, double K_I, double K_D) {
        Kp = K_P;
        Ki = K_I;
        Kd = K_D;
    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return error * Kp + derivative * Kd + integralSum * Ki;
    }
    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}
