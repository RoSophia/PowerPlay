package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.emd;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.RobotVars.rmd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
class PIDF implements Runnable {
    DcMotorEx motA, motB;

    public boolean shouldClose = false;
    public boolean use = true;
    public boolean curRet = false;
    public static boolean USE_TELE = true;
    public String name;
    public LinearOpMode lom;

    public double p;
    public double d;
    public double i;
    public double f;
    public double b;
    public double md;

    public PIDF(DcMotorEx motA, DcMotorEx motB, String n, double p, double d, double i, double f, double b, double md) {
        if (motA == null) {
            return;
        }
        this.motA = motA;
        /*
        motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        if (motB != null) {
            this.motB = motB;
            /*
            motB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        }
        this.p = p;
        this.d = d;
        this.i = i;
        this.f = f;
        this.b = b;
        this.md = md;
        this.name = n;
    }

    public void update_pid(double p, double d, double i, double f, double b) {
        if (motA == null) {
            return;
        }
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
        if (motA == null) {
            return;
        }
        ltarg = target;
        target = targ;
        DUR = tim;
        ttim.reset();
        integralSum = 0;
    }

    void updt() {
        if (motA == null) {
            return;
        }
        double x;
        if (DUR > 0.0001) {
            x = ttim.seconds() * (1 / DUR); /// Rescale the elapsed time to [0, 1]
        } else {
            x = 314;
        }
        if (USE_TELE) {
            TelemetryPacket cp = new TelemetryPacket();
            cp.put(name + "CurX", x);
            dashboard.sendTelemetryPacket(cp);
        }

        if (x <= 1) { /// If we have not yet reached the end of the movement, set the current target as `a*x*(1-x)^3 + b*(1-x)*x + x^3` from `ltarg` to `targ`
            //ctarg = ltarg + (int)(x * (target - ltarg));
            ctarg = ltarg + (int) ((A * x * (1 - x) * (1 - x) * (1 - x) + B * (1 - x) * x + x * x * x) * (target - ltarg));
        } else { /// We have already reached the destination and need not update any further
            ctarg = target;
        }
    }

    double error = 0;
    double derivate = 0;
    double lastError = 0;
    double integralSum = 0;
    double lastTarget = 0;

    double sign(double v) {
        if (v > 0) {
            return 1;
        } else if (v < 0) {
            return -1;
        } else {
            return 0;
        }
    }

    public static double CORRECTION = 0.13;

    public static int MAX_CURRENT_DRAW = 7000;
    public static double MAX_OVERCURRENT_TIME = 2.0;

    int lastExtPos = 0;
    ElapsedTime lastExtTim = new ElapsedTime(0);
    public static double EXT_RESET_TIME = 2.0;

    @SuppressWarnings("BusyWait")
    public void run() {
        if (motA == null) {
            return;
        }
        int cp;
        int cpb = 0;
        ElapsedTime timer2 = new ElapsedTime(0);
        lastTarget = target;
        ctarg = target;
        double outp = 0;
        ElapsedTime timer = new ElapsedTime(0);
        /*motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (motB != null) {
            motB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/
        boolean lf = false;
        ElapsedTime MOTATIMER = new ElapsedTime(0);
        MOTATIMER.reset();
        while (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) {
            cp = motA.getCurrentPosition();
            if (motB != null) {
                cpb = motB.getCurrentPosition();
            }
            if (USE_TELE) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put(name + "CycleTimeArm", timer2.milliseconds());
                timer2.reset();
                pack.put(name + "Target", target);
                pack.put(name + "lTarg", ltarg);
                pack.put(name + "cTarg", ctarg);
                pack.put(name + "ttim", ttim.seconds());
                pack.put(name + "Dur", DUR);
                pack.put(name + "CA", cp);
                if (motB != null) {
                    pack.put(name + "CB", cpb);
                }
                pack.put(name + "Power", outp);
                pack.put(name + "Error", error);
                pack.put(name + "Derivate", derivate);
                pack.put(name + "Isum", integralSum);
                pack.put(name + "LF", lf);
                pack.put(name + "use", use);
                pack.put(name + "curRet", curRet);
                pack.put(name + "PID_p", p);
                pack.put(name + "PID_i", i);
                pack.put(name + "PID_d", d);
                pack.put(name + "PID_f", f);
                pack.put(name + "LAST_EXT_POS", lastExtPos);
                pack.put(name + "LAST_EXT_TIM", lastExtTim);
                dashboard.sendTelemetryPacket(pack);
            }
            updt();
            if (use && !curRet) {
                if (cp < -10) {
                    motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    if (motB != null) {
                        motB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    lastError = 0;
                }
                if (lastTarget != target) {
                    lastTarget = target;
                }
                error = ctarg - cp;
                derivate = (error - lastError) / timer.seconds();
                integralSum = integralSum + (error * timer.seconds());

                outp = (p * error) + (d * derivate) + (i * integralSum) + f;

                //if (ctarg == target && (target < 100)) {
                if (ctarg == target) {
                    // THIS IS A HACK ON TOP OF ANOTHER HACK
                    // DO NOT ATTEMPT THIS AT HOME
                    if (Math.abs(ctarg - cp) < md) {
                        if (md == rmd && ctarg <= RBOT_POS + 1) {
                            outp = 0;
                        } else {
                            if (Math.abs(ctarg - cp) < md / 4) {
                                lf = true;
                            }
                            if (!lf) {
                                outp = CORRECTION * sign(ctarg - cp);
                            } else {
                                outp = f;
                            }
                        }
                    } else {
                        lf = false;
                    }
                }

                /// KILL MYSELF
                if (md == rmd && target == RBOT_POS && cp > 15) {
                    outp = -1;
                }

                /// KRILL MYSELF
                if (md == emd && target == EMIN && cp >= emd) {
                    if (lastExtTim.seconds() > EXT_RESET_TIME) {
                        TelemetryPacket tp = new TelemetryPacket();
                        tp.put("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", 1);
                        dashboard.sendTelemetryPacket(tp);
                        motA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        motA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        if (motB != null) {
                            motB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            motB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                    }
                } else {
                    lastExtPos = cp;
                    lastExtTim.reset();
                }

                if (motA.getCurrent(CurrentUnit.MILLIAMPS) < MAX_CURRENT_DRAW || motB.getCurrent(CurrentUnit.MILLIAMPS) < MAX_CURRENT_DRAW) {
                    MOTATIMER.reset();
                } else if (MOTATIMER.seconds() > MAX_OVERCURRENT_TIME) {
                    outp = 0;
                    target = ctarg = cp;
                }


                if (!shouldClose && !lom.isStopRequested() && lom.opModeIsActive()) { /// `lom` here is used to prevent powering the motor after the OpMode stopped.
                    motA.setPower(outp * pcoef);

                    if (motB != null) {
                        double bdif = (cp - cpb) * b;
                        if ((Math.abs(cp - cpb) < 4) || ctarg < 15) {
                            bdif = 0;
                        }
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
