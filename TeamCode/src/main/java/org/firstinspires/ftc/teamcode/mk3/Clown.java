package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.EMIN;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SAW;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.armHolding;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.conversiePerverssa;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.epd;
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.ext;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.List;

@SuppressWarnings("ALL")
@Config
public class Clown implements Runnable {
    public static int MIP = 100;
    public static double ME = 5;
    public static double ETC = 0.14;
    public static double CPT = 0.85;
    public static double CET = 0.5;
    public static double CHT = 0.75;
    public static double TTT = 0.0;
    public static boolean CLAW = false;
    public static double PUTC = 1.212;
    public static double PREPC = 0.8;
    public static double CIP = 0.11;
    public static double GB = 0.20;
    public static double GHT = 0.1;
    public static double CD = 0.15;
    public static double ED = 0.2;

    public static double ST = 0.4;
    public static double SD = 0.01;
    public int UST = 0;

    private Servo sa, sb, sHeading, sClaw, sBalans, sMClaw;
    private DcMotorEx ce;
    private boolean cput = false;
    private boolean cget = false;
    private boolean cprepCone = false;
    public boolean toPut = false;
    public boolean toGet = false;
    public boolean shouldClose = false;
    public boolean toPrepCone = false;
    public boolean tppc = false;
    public boolean cext = false;
    public boolean waiting = true;
    public boolean rtg = false;

    List<Double> tims = Arrays.asList(CPT, CET, CHT);
    int timt = 0;

    ElapsedTime et = new ElapsedTime(0);
    ElapsedTime ct = new ElapsedTime(0);

    public Clown(Servo sa, Servo sb, Servo sHeading, Servo sClaw, Servo sMClaw, Servo sBalans, DcMotorEx ce) {
        this.sa = sa;
        this.sb = sa;
        this.sHeading = sHeading;
        this.sClaw = sClaw;
        this.sMClaw = sMClaw;
        this.sBalans = sBalans;
        this.ce = ce;
    }

    double sp;
    double DT = 1.212;
    ElapsedTime gtim = new ElapsedTime(0);

    public void run() {
        while (!shouldClose) {
            if (USE_TELE) {
                TelemetryPacket packet = new TelemetryPacket();
                tims = Arrays.asList(CPT, CET, CHT);

                packet.put("cput", cput);
                packet.put("cget", cget);
                packet.put("cpre", cprepCone);
                packet.put("tpre", toPrepCone);
                packet.put("tput", toPut);
                packet.put("tget", toGet);
                packet.put("tppc", tppc);
                packet.put("cext", cext);
                packet.put("cred", coneReady);
                packet.put("ahol", armHolding);
                packet.put("ccla", coneClaw);
                packet.put("DT", DT);
                packet.put("rtg", rtg);
                packet.put("timt", timt);
                packet.put("ctim", tims.get(timt));

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            if (CLAW) {
                sClaw.setPosition(SINCHIS);
                CLAW = false;
                coneReady = true;
            }

            if (cput && toPut) {
                toPut = false;
            }

            if (coneClaw && toPut && ce.getCurrentPosition() < MIP) {
                tppc = false;
                toPut = false;
                cput = true;
                toPrepCone = false;
                cprepCone = false;
                cget = false;
                sp = sa.getPosition();

                conversiePerverssa(SAP);
                //sHeading.setPosition(SHP);
                sClaw.setPosition(SINCHIS);
                sMClaw.setPosition(SCO);
                sBalans.setPosition(SBAP);
                et.reset();
            }

            if (cput) {
                if (et.seconds() > TTT * DT) {
                    sHeading.setPosition(SHP);
                } else {
                    conversiePerverssa(SAP);
                    sClaw.setPosition(SINCHIS);
                    sMClaw.setPosition(SCO);
                    sBalans.setPosition(SBAP);
                }

                if (et.seconds() > tims.get(timt) * DT) {
                    sClaw.setPosition(SDESCHIS);
                    conversiePerverssa(SAW);
                }
                if (et.seconds() > (tims.get(timt) + CD) * DT) {
                    sMClaw.setPosition(SCC);
                }
                if (et.seconds() > (tims.get(timt) + ED) * DT) {
                    sBalans.setPosition(SBAG);
                    sClaw.setPosition(SDESCHIS);
                    sHeading.setPosition(SHG);
                    coneReady = true;
                    armHolding = false;
                    coneClaw = false;
                    toGet = false;
                    cget = false;
                    cput = false;
                    waiting = true;
                    timt = 0;
                    et.reset();
                }

            }

            if (toPrepCone) {
                sClaw.setPosition(SINCHIS);
                toPrepCone = false;
                cprepCone = true;
                cget = false;
                rtg = false;
                if (tppc) {
                    DT = PUTC;
                } else {
                    DT = PREPC;
                }
                ct.reset();
            }

            if (!coneClaw && !cprepCone && !toPrepCone && toPut) {
                tppc = true;
                toPrepCone = true;
                cget = false;
            }

            if (cprepCone && ct.seconds() > CIP) {
                cprepCone = false;
                conversiePerverssa(SAH);
                sBalans.setPosition(SBAH);
                armHolding = true;
                coneClaw = true;
                cget = false;
                if (!toPut) {
                    timt = 2;
                } else{
                    timt = 0;
                }
                if (epd.target > MIP && toPut) {
                    sHeading.setPosition(SHP);
                    sBalans.setPosition(SBAP);
                    conversiePerverssa(SAP);
                    ext(EMIN);
                    timt = 1;
                }
            }

            if (toGet) {
                tppc = false;
                cget = true;
                toPut = false;
                cput = false;
                coneClaw = false;
                armHolding = false;
                if (UST > 0) {
                    conversiePerverssa(ST - SD * (UST - 1));
                } else {
                    conversiePerverssa(SAG);
                }
                sBalans.setPosition(SBAG);
                sClaw.setPosition(SDESCHIS);
                toGet = false;
                rtg = true;
                gtim.reset();
            }

            if (cget && gtim.seconds() > GHT * DT) {
                sHeading.setPosition(SHG);
                cget = false;
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
