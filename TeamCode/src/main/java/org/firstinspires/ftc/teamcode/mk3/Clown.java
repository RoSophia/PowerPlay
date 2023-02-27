package org.firstinspires.ftc.teamcode.mk3;

import static org.firstinspires.ftc.teamcode.RobotVars.EMAX;
import static org.firstinspires.ftc.teamcode.RobotVars.IN_TESTING;
import static org.firstinspires.ftc.teamcode.RobotVars.SAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBAP;
import static org.firstinspires.ftc.teamcode.RobotVars.SBG;
import static org.firstinspires.ftc.teamcode.RobotVars.SBP;
import static org.firstinspires.ftc.teamcode.RobotVars.SCC;
import static org.firstinspires.ftc.teamcode.RobotVars.SCO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SHG;
import static org.firstinspires.ftc.teamcode.RobotVars.SHP;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_TELE;
import static org.firstinspires.ftc.teamcode.RobotVars.coneClaw;
import static org.firstinspires.ftc.teamcode.RobotVars.coneReady;

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
    public static double ETC = 0.3;

    private Servo sa, sb, sHeading, sClaw, sBalans, sMClaw;
    private DcMotorEx ce;
    private boolean cput = false;
    private boolean cget = false;
    public boolean toPut = false;
    public boolean toGet = false;
    public boolean shouldClose = true;

    boolean toOpenSC;
    ElapsedTime et = new ElapsedTime(0);

    public Clown(Servo sa, Servo sb, Servo sHeading, Servo sClaw, Servo sMClaw, Servo sBalans, DcMotorEx ce) {
        this.sa = sa;
        sa.setPosition(SAG);
        this.sb = sb;
        sb.setPosition(SBG);
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

    void upd_balans() {
        if (!IN_TESTING) {
            sBalans.setPosition(SBAG + ((sa.getPosition() - SAG) / (SAG - SAP)) * (SBAG - SBAP));
        }
    }

    public void run() {
        while (shouldClose) {
            if (USE_TELE) {
                TelemetryPacket packet = new TelemetryPacket();

                packet.put("toGet", toGet);
                packet.put("toPut", toPut);
                packet.put("cready", coneReady);
                packet.put("cclaw", coneClaw);
                packet.put("cput", cput);
                packet.put("toOpen", toOpenSC);
                packet.put("cget", cget);
                packet.put("sa", (sa.getPosition() - SAP) / (SAP - SAG));
                packet.put("sb", (sb.getPosition() - SBP) / (SBP - SBG));
                packet.put("sh", (sHeading.getPosition() - SHP) / (SHP - SHG));
                packet.put("sg", (sClaw.getPosition() - SINCHIS) / (SINCHIS - SDESCHIS));
                packet.put("sc", (sMClaw.getPosition() - SCC) / (SCC - SCO));

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            if ((cput || !coneClaw) && toPut) {
                toPut = false;
            }

            if (toPut && ce.getCurrentPosition() < MIP) {
                toPut = false;
                cput = true;
                sa.setPosition(SAP);
                sb.setPosition(SBP);
                sHeading.setPosition(SHP);
                sClaw.setPosition(SINCHIS);
                sMClaw.setPosition(SCC);
            }

            if (cput) {
                if (Math.abs(sa.getPosition() - SAP) < ME) {
                    sClaw.setPosition(SDESCHIS);
                    toOpenSC = true;
                    et.reset();
                    cput = false;
                }
            }

            if (toOpenSC && et.seconds() > ETC) {
                sMClaw.setPosition(SCO);
                coneReady = true;
                coneClaw = false;
                toOpenSC = false;
            }

            if (toGet) {
                cget = true;
                toGet = false;
                toPut = false;
                cput = false;
                sa.setPosition(SAG);
                sb.setPosition(SBG);
                sHeading.setPosition(SHG);
                sClaw.setPosition(SDESCHIS);
                sMClaw.setPosition(SCO);
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
