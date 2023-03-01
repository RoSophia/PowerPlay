package org.firstinspires.ftc.teamcode.mk3;

/*
         ____    _    ____ ___    _      _ _____  ___   ___
        |  _ \  / \  / ___|_ _|  / \    / |___ / / _ \ / _ \
        | | | |/ _ \| |    | |  / _ \   | | |_ \| | | | | | |
        | |_| / ___ \ |___ | | / ___ \  | |___) | |_| | |_| |
        |____/_/   \_\____|___/_/   \_\ |_|____/ \___/ \___/

         ____   ___  _____ _____      _    ____      _    ____  _____ ____   ____
        / ___| / _ \|  ___|_   _|    / \  |  _ \    / \  | __ )| ____/ ___| / ___|
        \___ \| | | | |_    | |     / _ \ | |_) |  / _ \ |  _ \|  _| \___ \| |
        ___) | |_| |  _|   | |    / ___ \|  _ <  / ___ \| |_) | |___ ___) | |___
        |____/ \___/|_|     |_|   /_/   \_\_| \_\/_/   \_\____/|_____|____/ \____|
        ____  _____ ______     _______ ____  ____
        |  _ \| ____|  _ \ \   / / ____|  _ \/ ___|
        | |_) |  _| | |_) \ \ / /|  _| | |_) \___ \
        |  __/| |___|  _ < \ V / | |___|  _ < ___) |
        |_|   |_____|_| \_\ \_/  |_____|_| \_\____/

         _____   ____  _____ ______     _______ ____  ____
        | ____| |  _ \| ____|  _ \ \   / / ____|  _ \/ ___|
        |  _|   | |_) |  _| | |_) \ \ / /|  _| | |_) \___ \
        | |___  |  __/| |___|  _ < \ V / | |___|  _ < ___) |
        |_____| |_|   |_____|_| \_\ \_/  |_____|_| \_\____/
         ____  _   _ ____  _____      _    ____  _____ ____  _____ _   _ _____  _
        |  _ \| | | |  _ \| ____|    / \  |  _ \| ____|  _ \| ____| \ | |_   _|/ \
        | |_) | | | | |_) |  _|     / _ \ | | | |  _| | |_) |  _| |  \| | | | / _ \
        |  _ <| |_| |  __/| |___   / ___ \| |_| | |___|  _ <| |___| |\  | | |/ ___ \
        |_| \_\\___/|_|   |_____| /_/   \_\____/|_____|_| \_\_____|_| \_| |_/_/   \_\
 */

/*
 * SOFT ARABESC PERVERS VERSIUNE 0.31δ
 * AUTOR: VERICU
 * E PERVERS, RUPE ADERENTA
 *
 * Cu de toate fara ceapa boss
 */

import static org.firstinspires.ftc.teamcode.RobotVars.CU_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.RBOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RMID_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RMIU_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.RTOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.S1PO;
import static org.firstinspires.ftc.teamcode.RobotVars.S2PO;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.RobotVars.ebp;
import static org.firstinspires.ftc.teamcode.RobotVars.ed;
import static org.firstinspires.ftc.teamcode.RobotVars.ef;
import static org.firstinspires.ftc.teamcode.RobotVars.ei;
import static org.firstinspires.ftc.teamcode.RobotVars.ep;
import static org.firstinspires.ftc.teamcode.RobotVars.pcoef;
import static org.firstinspires.ftc.teamcode.RobotVars.rbp;
import static org.firstinspires.ftc.teamcode.RobotVars.rd;
import static org.firstinspires.ftc.teamcode.RobotVars.rf;
import static org.firstinspires.ftc.teamcode.RobotVars.ri;
import static org.firstinspires.ftc.teamcode.RobotVars.rp;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.S1;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.S2;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.batteryVoltageSensor;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.clo;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.extB;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.imu;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rid;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ridA;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.rpd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sBalans;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sClose;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sHeading;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.sMCLaw;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.spe;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.startma;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings({"SpellCheckingInspection"})
@Config
@TeleOp
public class OP_Mode_mk3 extends LinearOpMode {
    boolean L2A = false;
    boolean L2RB = false;
    boolean L2B = false;
    boolean L2Y = false;
    boolean L2U = false;
    boolean L2D = false;
    boolean G2X = false;
    boolean R2RB = false;
    boolean R2LB = false;
    boolean R2LT = false;
    boolean RB = false;
    boolean switched = false;

    public static double headP = 1.2;

    public static double UPPS = 100;
    double UPP = 100;
    public static double UPPP = 100;

    public double lep, led, lei, lef, lebp;
    public double rep, red, rei, ref, rebp;

    public static double P1 = 1;
    public static double P2 = 1;
    public static double P3 = 1;
    public static double P4 = 1;

    public double WT = 0.2;
    public static double HMIN = 0.005;

    public void runOpMode() {
        L2A = L2B = L2Y = L2U = L2D = G2X = R2RB = R2LB = R2LT = RB = switched = coneReady = false;
        UPP = UPPS;

        initma(hardwareMap);

        waitForStart();

        startma();

        ElapsedTime timer = new ElapsedTime(0);
        ElapsedTime g1t = new ElapsedTime(0);
        double lhpos = 0;
        lep = ep;
        lei = ei;
        led = ed;
        lef = ef;
        lebp = ebp;
        rep = rp;
        rei = ri;
        red = rd;
        ref = rf;
        rebp = rbp;
        while (opModeIsActive()) {
            if (lep != ep || lei != ei || led != ed || lef != ef || lebp != ebp) {
                epd.update_pid(ep, ei, ed, ef, ebp);
                lep = ep;
                lei = ei;
                led = ed;
                lebp = ebp;
            }
            if (rep != rp || rei != ri || red != rd || ref != rf || rebp != rbp) {
                rpd.update_pid(rp, ri, rd, rf, rbp);
                rep = rp;
                rei = ri;
                red = rd;
                rebp = rbp;
            }

            if (USE_TELE) {
                TelemetryPacket fp = new TelemetryPacket();
                fp.put("CycleTime", timer.milliseconds());
                fp.put("Orient", imu.getAngularOrientation());
                timer.reset();
                dashboard.sendTelemetryPacket(fp);
            }

            if (CU_TESTING) {
                conversiePerverssa(SAG);
                sClose.setPosition(SINCHIS);
                sHeading.setPosition(SHG);
                sMCLaw.setPosition(SCC);
                sBalans.setPosition(SBAG);
                S1.setPosition(S1PO);
                S2.setPosition(S2PO);
            }

            final double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double ch = imu.getAngularOrientation().firstAngle;
            final double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;// + ch;
            double hdif = 0;
            if (Math.abs(gamepad1.right_stick_x) > 0.001) {
                g1t.reset();
            }
            if (g1t.seconds() < WT) {
                lhpos = ch;
            } else {
                hdif = lhpos - ch;
                if (hdif > Math.PI / 2) {
                    hdif = Math.PI - hdif;
                } else if (hdif < -Math.PI / 2) {
                    hdif -= Math.PI;
                }
                if (Math.abs(hdif) < HMIN || speed > 0.01) {
                    hdif = 0;
                }
            }

            final double turn = -hdif * headP + gamepad1.right_stick_x;
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);

            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;

            if (!L2RB && gamepad2.right_bumper) {
                UPP += UPPP;
            }
            L2RB = gamepad2.right_bumper;

            if (!L2A && gamepad2.a) {
                rid(RTOP_POS);
            }
            L2A = gamepad2.a;
            if (!L2U && gamepad2.dpad_up) {
                rid(RMID_POS);
            }
            L2U = gamepad2.dpad_up;
            if (!L2D && gamepad2.dpad_down) {
                rid(RMIU_POS);
            }
            L2D = gamepad2.dpad_down;
            if (!L2B && gamepad2.b) {
                clo.toGet = true;
                rid(RBOT_POS);
                ext(EMIN);
            }
            L2B = gamepad2.b;

            if (!L2Y && gamepad2.y) {
                clo.toPut = true;
            }
            L2Y = gamepad2.y;

            if (!R2LB && gamepad2.left_bumper) {
                clo.toPrepCone = true;
            }
            R2LB = gamepad2.left_bumper;

            if (!R2RB && gamepad2.right_bumper) {
                ext(EMAX);
            }
            R2RB = gamepad2.right_bumper;

            final double DPC = 1 - 0.6 * gamepad2.right_trigger;
            if (Math.abs(gamepad2.right_stick_y) > 0.001) {
                spe(false, -gamepad2.right_stick_y * DPC);
                epd.set_target(extA.getCurrentPosition(), 0);
            } else {
                spe(false, 0);
            }

            if (Math.abs(gamepad2.left_stick_y) > 0.001) {
                spe(true, -gamepad2.left_stick_y * DPC);
                rpd.set_target(ridA.getCurrentPosition(), 0);
            } else {
                spe(true, 0);
            }

            if (!G2X && gamepad2.x) {
                if (switched) {
                    sClose.setPosition(SDESCHIS);
                } else {
                    sClose.setPosition(SINCHIS);
                }
                switched = !switched;
            }
            G2X = gamepad2.x;

            if (USE_TELE) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("extA", extA.getCurrentPosition());
                pack.put("extB", extB.getCurrentPosition());
                pack.put("ridA", ridA.getCurrentPosition());
                pack.put("ch", ch);
                dashboard.sendTelemetryPacket(pack);
            }

            pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            final double fcoef = pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef * P1);
            rightFront.setPower(rfPower * fcoef * P2);
            leftBack.setPower(lbPower * fcoef * P3);
            rightBack.setPower(rbPower * fcoef * P4);
        }

        endma();
    }
}
