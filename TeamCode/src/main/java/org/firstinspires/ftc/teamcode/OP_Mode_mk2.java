package org.firstinspires.ftc.teamcode;

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
 * SOFT ARABESC PERVERS VERSIUNE 0.24c
 * AUTOR: VERICU
 * E PERVERS, RUPE ADERENTA
 *
 * Cu de toate fara ceapa boss
 */

import static org.firstinspires.ftc.teamcode.RobotConstants.BOT_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.CSM;
import static org.firstinspires.ftc.teamcode.RobotConstants.DIFP;
import static org.firstinspires.ftc.teamcode.RobotConstants.DOT;
import static org.firstinspires.ftc.teamcode.RobotConstants.FC;
import static org.firstinspires.ftc.teamcode.RobotConstants.MID_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.MINP;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIU_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.P1;
import static org.firstinspires.ftc.teamcode.RobotConstants.P2;
import static org.firstinspires.ftc.teamcode.RobotConstants.P3;
import static org.firstinspires.ftc.teamcode.RobotConstants.P4;
import static org.firstinspires.ftc.teamcode.RobotConstants.S1PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S2PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S3PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.SPC;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.UPPP;
import static org.firstinspires.ftc.teamcode.RobotConstants.UPPS;
import static org.firstinspires.ftc.teamcode.RobotConstants.UPT;
import static org.firstinspires.ftc.teamcode.RobotConstants.USE_TELEMETRY;
import static org.firstinspires.ftc.teamcode.RobotConstants.dif;
import static org.firstinspires.ftc.teamcode.RobotConstants.pcoef;
import static org.firstinspires.ftc.teamcode.RobotConstants.rd;
import static org.firstinspires.ftc.teamcode.RobotConstants.rf;
import static org.firstinspires.ftc.teamcode.RobotConstants.ri;
import static org.firstinspires.ftc.teamcode.RobotConstants.rp;
import static org.firstinspires.ftc.teamcode.RobotFuncs.S1;
import static org.firstinspires.ftc.teamcode.RobotFuncs.S2;
import static org.firstinspires.ftc.teamcode.RobotFuncs.S3;
import static org.firstinspires.ftc.teamcode.RobotFuncs.armRun;
import static org.firstinspires.ftc.teamcode.RobotFuncs.batteryVoltageSensor;
import static org.firstinspires.ftc.teamcode.RobotFuncs.cs;
import static org.firstinspires.ftc.teamcode.RobotFuncs.dashboard;
import static org.firstinspires.ftc.teamcode.RobotFuncs.endma;
import static org.firstinspires.ftc.teamcode.RobotFuncs.imu;
import static org.firstinspires.ftc.teamcode.RobotFuncs.initma;
import static org.firstinspires.ftc.teamcode.RobotFuncs.leftBack;
import static org.firstinspires.ftc.teamcode.RobotFuncs.leftFront;
import static org.firstinspires.ftc.teamcode.RobotFuncs.rid;
import static org.firstinspires.ftc.teamcode.RobotFuncs.ridicareSlide;
import static org.firstinspires.ftc.teamcode.RobotFuncs.rightBack;
import static org.firstinspires.ftc.teamcode.RobotFuncs.rightFront;
import static org.firstinspires.ftc.teamcode.RobotFuncs.s1;
import static org.firstinspires.ftc.teamcode.RobotFuncs.startma;
import static org.firstinspires.ftc.teamcode.RobotFuncs.underglow;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class OP_Mode_mk2 extends LinearOpMode {
    int poz = 0;

    boolean L2A = false;
    boolean L2RB = false;
    boolean L2B = false;
    boolean L2U = false;
    boolean L2D = false;
    boolean G2X = false;
    boolean R2RB = false;
    boolean R2LB = false;
    boolean R2LT = false;
    boolean RB = false;
    boolean switched = false;

    double UPP = 100;
    double luv = 0;

    ElapsedTime et = new ElapsedTime(0);
    ElapsedTime lmv = new ElapsedTime(0);
    public static double MIT = 0.05;

    void upd_underglow(double speed) { /// Underglow cam pervers
        double cs = Math.abs(speed) * Math.sqrt(2) / 2 * SPC;
        if (cs > luv) {
            luv = cs;
        } else {
            luv -= et.seconds() * FC;
        }
        luv = Math.min(Math.max(luv, MINP), 1);

        underglow.setPower(-luv);
        et.reset();
    }

    public static double XOEF = 1.0;
    public static double YOEF = 1.0;

    double lrp, lri, lrd, lrf;
    void st(int p) {
        armRun.set_target(p, (p != BOT_POS) ? UPT : DOT);
    }

    public void runOpMode() {
        initma(hardwareMap);

        L2A = L2B = L2U = L2D = G2X = R2RB = R2LB = R2LT = RB = switched = false;
        UPP = UPPS;

        waitForStart();

        startma(this);

        ElapsedTime timer = new ElapsedTime(0);
        lrp = rp;
        lri = ri;
        lrd = rd;
        lrf = rf;
        while (opModeIsActive()) {
            if (lrp != rp || lri != ri || lrd != rd || lrf != rf) {
                armRun.update_pid(rp, rd, ri, rf);
                lrp = rp;
                lri = ri;
                lrd = rd;
                lrf = rf;
            }
            S1.setPosition(S1PO);
            S2.setPosition(S2PO);
            S3.setPosition(S3PO);

            if (!L2RB && gamepad2.right_bumper) {
                UPP += UPPP;
            }
            L2RB = gamepad2.right_bumper;

            if (!L2A && gamepad2.a) {
                st(TOP_POS + (int) (gamepad2.left_trigger * UPP));
            }
            L2A = gamepad2.a;

            if (!L2U && gamepad2.dpad_up) {
                st(MIU_POS + (int) (gamepad2.left_trigger * UPP));
            }
            L2U = gamepad2.dpad_up;

            if (!L2D && gamepad2.dpad_down) {
                st(MID_POS + (int) (gamepad2.left_trigger * UPP));
            }
            L2D = gamepad2.dpad_down;

            if (!L2B && gamepad2.b) {
                st(BOT_POS + (int) (gamepad2.left_trigger * UPP));
            }
            L2B = gamepad2.b;

            final double DPC = 1 - 0.6 * gamepad2.right_trigger;
            if (Math.abs(gamepad2.right_stick_y) > 0.001) {
                rid(-gamepad2.right_stick_y * DPC);
            } else {
                rid(0);
            }

            if (!RB && gamepad2.left_bumper && !gamepad2.right_bumper) {
                armRun.set_target(armRun.target - dif, 0);
            }
            if (RB && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                armRun.set_target(armRun.target + dif, 0);
            }
            RB = gamepad2.left_bumper;

            if (!G2X && gamepad2.x) {
                if (switched) {
                    s1.setPosition(SDESCHIS);
                    switched = false;
                } else {
                    final double cd = cs.getDistance(DistanceUnit.MM);
                    if (cd < 100) {
                        s1.setPosition(SINCHIS + cs.getDistance(DistanceUnit.MM) * CSM);
                        switched = true;
                    }
                }
            }
            G2X = gamepad2.x;

            if (USE_TELEMETRY) {
                TelemetryPacket pack = new TelemetryPacket();
                pack.put("Acc", imu.getCalibrationStatus());
                pack.put("cs", cs.getDistance(DistanceUnit.MM));
                pack.put("Ridicare", ridicareSlide.getCurrentPosition());
                pack.put("Cpow", ridicareSlide.getPower());
                pack.put("left_stick_y", poz);
                pack.put("r_stick_y", gamepad2.right_stick_y);
                pack.put("pad", gamepad1.right_trigger);
                pack.put("CycleTime", timer.milliseconds());
                timer.reset();
                dashboard.sendTelemetryPacket(pack);
            }

            final double speed = Math.hypot(gamepad1.left_stick_x * XOEF, gamepad1.left_stick_y * YOEF);
            final double angle = Math.atan2(gamepad1.left_stick_y * YOEF, gamepad1.left_stick_x * XOEF) - Math.PI / 4;
            final double turn = -gamepad1.right_stick_x;
            if ((speed == 0) && (turn == 0)) {
                lmv.reset();
            }
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);
            //maths (nu stiu eu deastea ca fac cu antohe)
            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;
            upd_underglow(Math.abs(turn) + Math.abs(speed));

            //dam blana in motoare
            pcoef = 14.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            final double scoef = lmv.time() > MIT ? 1.0 : lmv.time() / MIT;
            final double fcoef = pcoef * spcoef * scoef;
            leftFront.setPower(lfPower * fcoef * P1);  // LB
            rightFront.setPower(rfPower * fcoef * P2); // LF
            leftBack.setPower(lbPower * fcoef * P3);   // RB
            rightBack.setPower(rbPower * fcoef * P4);  // RF
        }
        endma();
    }
}
