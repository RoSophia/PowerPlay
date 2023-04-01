package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
class PIDF implements Runnable {
    DcMotorEx motA, motB;

    public boolean shouldClose = false;
    public boolean use = true;
    public boolean useTele = true;
    public String name;
    public LinearOpMode lom;

    public double p;
    public double d;
    public double i;
    public double f;
    public double b;

    public PIDF(DcMotorEx motA, DcMotorEx motB, String n, double p, double d, double i, double f, double b) {
        this.motA = motA;
        motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motB != null) {
            this.motB = motB;
            motB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        this.p = p;
        this.d = d;
        this.i = i;
        this.f = f;
        this.b = b;
        this.name = n;
    }

    public void update_pid(double p, double d, double i, double f, double b) {
        this.p = p;
        this.d = d;
        this.i = i;
        this.f = f;
        this.b = b;
    }

    int target = 0;
    int ctarg = 0;
    int ltarg = 0;

    public static double A = 0.1;
    public static double B = 2.9;
    double DUR = 1;

    ElapsedTime ttim = new ElapsedTime(0);
    public void set_target(int targ, double tim) { /// Start a new movement from `target` to `targ` in `tim` time. The actual calculations are done in `updt()`
        TelemetryPacket cp = new TelemetryPacket();
        cp.put(name + "GoToLpos", target);
        cp.put(name + "GoToCpos", targ);
        cp.put(name + "GoToTime", tim);
        dashboard.sendTelemetryPacket(cp);
        ltarg = target;
        target = targ;
        DUR = tim;
        ttim.reset();
    }

    void updt() {
        double x = ttim.seconds() * (1 / DUR); /// Rescale the elapsed time to [0, 1]
        TelemetryPacket cp = new TelemetryPacket();
        cp.put(name + "CurX", x);
        dashboard.sendTelemetryPacket(cp);

        if (x <= 1) { /// If we have not yet reached the end of the movement, set the current target as `a*x*(1-x)^3 + b*(1-x)*x + x^3` from `ltarg` to `targ`
            //ctarg = ltarg + (int)(x * (target - ltarg));
            ctarg = ltarg + (int)((A * x * (1 - x) * (1 - x) * (1 - x) + B * (1 - x) * x + x * x * x) * (target - ltarg));
        } else { /// We have already reached the destination and need not update any further
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
        ElapsedTime timer = new ElapsedTime(0);
        motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motB != null) {
            motB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        while (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) {
            if (useTele) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put(name + "CycleTimeArm", timer2.milliseconds());
                timer2.reset();
                pack.put(name + "Target", target);
                pack.put(name + "lTarg", ltarg);
                pack.put(name + "cTarg", ctarg);
                pack.put(name + "ttim", ttim.seconds());
                pack.put(name + "Dur", DUR);
                pack.put(name + "CA", motA.getCurrentPosition());
                if (motB != null) {
                    pack.put(name + "CB", motB.getCurrentPosition());
                }
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
                if (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) { /// `lom` here is used to prevent powering the motor after the OpMode stopped.
                    motA.setPower(outp * pcoef);

                    if (motB != null) {
                        double bdif = (motA.getCurrentPosition() - motB.getCurrentPosition()) * b;
                        motB.setPower((outp + bdif) * pcoef);
                    }
                } else {
                    motA.setPower(0);
                    if (motB != null) {
                        motB.setPower(0);
                    }
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
        if (motB != null) {
            motB.setPower(0);
        }
    }
}
