package org.firstinspires.ftc.teamcode.mk3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class test extends LinearOpMode {
    public static double ccp = 0;
    public Servo ccs, ccs2 = null;
    public static int CT = 1;
    public static boolean CC = false;
    public static double OFF = 0.27;
    public static double OFP = 0.01;
    public void runOpMode() {
        if ((CT & 1) != 0) {
            ccs = hardwareMap.get(Servo.class, "sClose");
        } else if ((CT & 2) != 0) {
            ccs = hardwareMap.get(Servo.class, "sHeading");
        } else if ((CT & 4) != 0) {
            ccs = hardwareMap.get(Servo.class, "sBalans");
        } else if ((CT & 8) != 0) {
            ccs = hardwareMap.get(Servo.class, "sMCLaw");
        } else if ((CT & 16) != 0) {
            ccs = hardwareMap.get(Servo.class, "sextA");
            if (CC) {
                ccs2 = hardwareMap.get(Servo.class, "sextB");
            }
        } else if ((CT & 32) != 0) {
            ccs = hardwareMap.get(Servo.class, "sextB");
        } else {
            ccs = hardwareMap.get(Servo.class, "Toate");
        }

        waitForStart();

        while (opModeIsActive()) {
            ccs.setPosition(ccp);
            if (ccs2 != null) {
                ccs2.setPosition(1 - ccp + OFF + (1 - ccp) * OFP);
            }
            TelemetryPacket pack = new TelemetryPacket();
            pack.put("ccp", ccp);
            pack.put("csp", 1 - ccp + OFF);
            FtcDashboard.getInstance().sendTelemetryPacket(pack);
        }

    }
}