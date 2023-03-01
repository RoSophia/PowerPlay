package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAH;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
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
import static org.firstinspires.ftc.teamcode.mk3.RobotFuncs.imu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("ALL")
@Config
public class Clown implements Runnable {
    public static int MIP = 100;
    public static double ME = 0.01;
    public static double ETC = 0.14;
    public static double CPT = 0.7;
    public static double TTT = 0.3;
    public static boolean CLAW = false;
    public static double PUTC = 1.212;
    public static double PREPC = 0.8;
    public static double CIP = 0.11;

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

    boolean toCloseSC;
    ElapsedTime et = new ElapsedTime(0);
    ElapsedTime ct = new ElapsedTime(0);

    public Clown(Servo sa, Servo sb, Servo sHeading, Servo sClaw, Servo sMClaw, Servo sBalans, DcMotorEx ce) {
        conversiePerverssa(SAG);
        this.sa = sa;
        this.sb = sa;
        this.sHeading = sHeading;
        sHeading.setPosition(SHG);
        this.sClaw = sClaw;
        sClaw.setPosition(SDESCHIS);
        this.sMClaw = sMClaw;
        sMClaw.setPosition(SCO);
        this.sBalans = sBalans;
        sBalans.setPosition(SBAG);
        this.ce = ce;
    }

    double sp;
    double DT = 1.212;
    public void run() {
        while (!shouldClose) {
            if (USE_TELE) {
                TelemetryPacket packet = new TelemetryPacket();

                packet.put("cput", cput);
                packet.put("cget", cget);
                packet.put("cpre", cprepCone);
                packet.put("tpre", toPrepCone);
                packet.put("tput", toPut);
                packet.put("tget", toGet);
                packet.put("tppc", tppc);
                packet.put("DT", DT);

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
                conversiePerverssa(SAP);
                sClaw.setPosition(SINCHIS);
                sMClaw.setPosition(SCO);
                sBalans.setPosition(SBAP);
                if (et.seconds() > TTT * DT) {
                    sHeading.setPosition(SHP);
                }
                if (et.seconds() > CPT * DT) {
                    sClaw.setPosition(SDESCHIS);
                    toCloseSC = true;
                    et.reset();
                    cput = false;
                }
            }

            if (toPrepCone) {
                sClaw.setPosition(SINCHIS);
                toPrepCone = false;
                cprepCone = true;
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
            }


            if (cprepCone && ct.seconds() > CIP) {
                cprepCone = false;
                conversiePerverssa(SAH);
                sBalans.setPosition(SBAH);
                coneClaw = true;
            }

            if (toCloseSC && et.seconds() > ETC * DT) {
                sMClaw.setPosition(SCC);
                coneReady = true;
                armHolding = false;
                coneClaw = false;
                toCloseSC = false;
                toGet = true;
            }

            if (toGet) {
                if (sa.getPosition() != SAG || sHeading.getPosition() != SHG || sBalans.getPosition() != SBAG) {
                    tppc = false;
                    cget = true;
                    toPut = false;
                    cput = false;
                    conversiePerverssa(SAG);
                    sHeading.setPosition(SHG);
                    sBalans.setPosition(SBAG);
                }
                sClaw.setPosition(SDESCHIS);
                toGet = false;
            }

            if (cget) {
                if (Math.abs(ce.getCurrentPosition() - EMAX) < ME) {
                    cget = false;
                }
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
