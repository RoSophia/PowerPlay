package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
class ThreadInfo {
    public static int target = 0;
    public static boolean shouldClose = false;
    public static boolean use = true;
    public static int fr;
    public static boolean useTele = false;
}

@Config
class ArmcPIDF implements Runnable {
    DcMotorEx ridicareSlide;

    public ArmcPIDF(DcMotorEx ridicareSlide) {
        this.ridicareSlide = ridicareSlide;
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static double pd = 0.001;
    public static double pu = 0.01;
    public static double d = 0;
    public static double i = 0.0001;
    public static double Kf = 0.001;

    public static double LPC = 2.5;

    double error = 0;
    double derivate = 0;
    double lastError = 0;
    double integralSum = 0;

    @SuppressWarnings("BusyWait")
    public void run() {
        ElapsedTime timer2 = new ElapsedTime();
        double outp = 0;
        timer2.reset();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!ThreadInfo.shouldClose) {
            if (ThreadInfo.useTele) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("CycleTimeArm", timer2.milliseconds());
                timer2.reset();
                pack.put("Target", ThreadInfo.target);
                pack.put("Current", ridicareSlide.getCurrentPosition());
                pack.put("Power", outp);
                dashboard.sendTelemetryPacket(pack);
            }
            if (ThreadInfo.use) {
                error = ThreadInfo.target - ridicareSlide.getCurrentPosition();
                //derivate = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                if (error < -220) {
                    outp = /*-(ppd * error * error) +*/ (pd * error) + (d * derivate) + (i * integralSum) + Kf;
                } else {
                    outp = /*(ppu * error * error) +*/ (pu * error) + (d * derivate) + (i * integralSum) + Kf;
                }
                if (ThreadInfo.target < 150 && ridicareSlide.getCurrentPosition() < 150) {
                    outp /= LPC;
                }
                ridicareSlide.setPower(outp);

                lastError = error;
                timer.reset();
            } else {
                error = derivate = lastError = integralSum = 0;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        ridicareSlide.setPower(0);
    }
}
