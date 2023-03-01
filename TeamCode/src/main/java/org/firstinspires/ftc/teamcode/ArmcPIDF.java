package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

class ThreadInfo {
    public static int target = 0;
    public static boolean shouldClose = false;
    public static boolean use = true;
    public static boolean fast = false;
    public static int fr;
    public static boolean useTele = false;
    public static double pcoef;
}

class ArmcPIDF implements Runnable {
    DcMotorEx ridicareSlide;

    public ArmcPIDF(DcMotorEx ridicareSlide) {
        this.ridicareSlide = ridicareSlide;
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public static double pd = 0.01;
    public static double pf = 0.02;
    public static double pu = 0.01;
    public static double d = 0.00002;
    public static double dd = 0.00001;
    public static double i = 0.0000;
    public static double Kfu = 0.001;
    public static double Kfd = 0.001;

    public static double accu = 50000;
    public static double accd = 3000;
    public static double accdd = 100000;
    public static double MINS = 1000;

    public static double LPC = 1.7;

    double error = 0;
    double derivate = 0;
    double lastError = 0;
    double integralSum = 0;
    double lastTarget = 0;
    double ctarg = 0;

    @SuppressWarnings("BusyWait")
    public void run() {
        ElapsedTime timer2 = new ElapsedTime(0);
        lastTarget = ThreadInfo.target;
        ctarg = ThreadInfo.target;
        double outp = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime(0);
        ElapsedTime tt = new ElapsedTime(0);
        while (!ThreadInfo.shouldClose) {
            if (ThreadInfo.useTele) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("CycleTimeArm", timer2.milliseconds());
                timer2.reset();
                pack.put("Target", ThreadInfo.target);
                pack.put("cTarg", ctarg);
                pack.put("Current", ridicareSlide.getCurrentPosition());
                pack.put("Power", outp);
                dashboard.sendTelemetryPacket(pack);
            }
            if (ThreadInfo.use) {
                if (lastTarget != ThreadInfo.target) {
                    lastTarget = ThreadInfo.target;
                }

                if (ctarg > ThreadInfo.target) {
                    double cd = Math.min(-timer.seconds() * accd + timer.seconds() * timer.seconds() * accdd, -MINS * timer.seconds());
                    ctarg = Math.max(ctarg + cd, ThreadInfo.target);
                } else {
                    ctarg = Math.min(ctarg + timer.seconds() * accu, ThreadInfo.target);
                }
                error = ctarg - ridicareSlide.getCurrentPosition();
                derivate = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());
                if (ThreadInfo.fast) {
                    ctarg = ThreadInfo.target;
                    outp = (pf * error) + (dd * derivate) + Kfu;
                } else {
                    if (error < 0) {
                        outp = /*-(ppd * error * error) +*/ (pd * error) + (d * derivate) + (i * integralSum) + Kfd;
                    } else {
                        outp = /*(ppu * error * error) +*/ (pu * error) + (d * derivate) + (i * integralSum) + Kfd;
                    }
                }
                if (ctarg < 150 && ridicareSlide.getCurrentPosition() < 150) {
                    outp /= LPC;
                }
                ridicareSlide.setPower(outp * ThreadInfo.pcoef);

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
