package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp, Ki, Kd;
    private double integralSum = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    /**
     * Creates a PIDController object for given PID constants
     * @param K_P The PID constant for proportional
     * @param K_I The PID constant for integral
     * @param K_D The PID constant for derivative
     */
    public PIDController(double K_P, double K_I, double K_D) {
        Kp = K_P;
        Ki = K_I;
        Kd = K_D;
    }

    /**
     * Calculates the power to reach the reference using PID algorithm
     * @param reference the target position for the motor
     * @param state the current position of the motor
     * @return the desired power, between -1.0 and 1.0
     */
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        return error * Kp + derivative * Kd + integralSum * Ki;
    }

    /**
     * Resets the integral sum for a new PID path
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}
