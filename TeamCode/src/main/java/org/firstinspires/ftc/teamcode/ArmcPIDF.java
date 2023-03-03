package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConstants.pcoef;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
class PIDF implements Runnable {
    DcMotorEx motA;

    public boolean shouldClose = false;
    public boolean use = true;
    public boolean useTele = true;
    public String name;
    public LinearOpMode lom;

    public double p;
    public double d;
    public double i;
    public double f;

    public PIDF(DcMotorEx motA, String n, double p, double d, double i, double f) {
        this.motA = motA;
        motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.p = p;
        this.d = d;
        this.i = i;
        this.f = f;
        this.name = n;
    }

    public void update_pid(double p, double d, double i, double f) {
        this.p = p;
        this.d = d;
        this.i = i;
        this.f = f;
    }

    int target = 0;
    int ctarg = 0;
    int ltarg = 0;

    public static double A = 0.1;
    public static double B = 2.9;
    public static double DUR = 1;

    ElapsedTime ttim = new ElapsedTime(0);
    public void set_target(int t, double T) {
        ltarg = target;
        target = t;
        DUR = T;
        ttim.reset();
    }

    void updt() {
        double x = ttim.seconds() * (1 / DUR);
        if (x <= 1) {
            //ctarg = ltarg + (int)(x * (target - ltarg));
            ctarg = ltarg + (int)((A * x * (1 - x) * (1 - x) * (1 - x) + B * (1 - x) * x + x * x * x) * (target - ltarg));
        } else {
            ctarg = target;
        }
    }

    double error = 0;
    double derivate = 0;
    double lastError = 0;
    double integralSum = 0;
    double lastTarget = 0;

    @SuppressWarnings("BusyWait")
    public void run() {
        ElapsedTime timer2 = new ElapsedTime(0);
        lastTarget = target;
        ctarg = target;
        double outp = 0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        ElapsedTime timer = new ElapsedTime(0);
        while (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) {
            if (useTele) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put(name + "CycleTimeArm", timer2.milliseconds());
                timer2.reset();
                pack.put(name + "Target", target);
                pack.put(name + "lTarg", ltarg);
                pack.put(name + "cTarg", ctarg);
                pack.put(name + "ttim", ttim.seconds());
                pack.put(name + "CA", motA.getCurrentPosition());
                pack.put(name + "Power", outp);
                dashboard.sendTelemetryPacket(pack);
            }
            updt();
            if (use) {
                if (lastTarget != target) {
                    lastTarget = target;
                }
                error = ctarg - motA.getCurrentPosition();
                derivate = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());

                outp = (p * error) + (d * derivate) + (i * integralSum) + f;
                if (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) {
                    motA.setPower(outp * pcoef);
                } else {
                    motA.setPower(0);
                }

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
        motA.setPower(0);
    }
}
