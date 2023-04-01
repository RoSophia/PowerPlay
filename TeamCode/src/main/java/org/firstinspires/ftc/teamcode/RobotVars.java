package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotVars {
    public static double SDESCHIS = 0.95; // Claw Open
    public static double SINCHIS = 0.8;   // Claw Closed

    public static int EMIN = -3;          // Minimum Extension
    public static int EMAX = 595;         // Maximum Extension

    public static int RTOP_POS = 537;     // Maximum Lift Position
    public static int RMIU_POS = 336;     // Medium Lift Position
    public static int RMID_POS = 122;     // Low Lift Position
    public static int RBOT_POS = 0;       // Bottom Lift position
    public static int LEEW = 50;          // Leeway For Manual Lift Movement

    public static double SAG = 0.4735;    // Grabber Arm Get position
    public static double SAH = 0.415;     // Grabber Arm Hold Position
    public static double SAP = 0.385;     // Grabber Arm Put position
    public static double SAW = 0.4;       // Grabber Arm Wait position

    public static double SBAG = 1.00;     // 4bar Get position
    public static double SBAH = 1.00;     // 4bar Hold position
    public static double SBAP = 0.60;     // 4bar Put position

    public static double SDIF = 0.276;    // Grabber Arm Servo Diff Term
    public static double SDIP = 0.01;     // Grabber Arm Servo Proportional Drift Term

    public static double SHG = 0.1;       // Grabber Get Heading
    public static double SHP = 0.76;      // Grabber Put Heading

    public static double SCO = 0.0;       // Cone Securing Mini Servo Close Position
    public static double SCC = 0.44;      // Cone Securing Mini Servo Open Position 

    public static double DOT = 2.1;       // Time To Lower The Lift
    public static double UPT = 0.0;       // Time To Hoist The Lift
    public static double EXTT = 0.5;      // Time To Fully Extend
    public static double RETT = 0.6;      // Time To Retract

    public static boolean USE_PHOTON = true;

    public static boolean USE_TELE = true; // Use Telemetry
    public static boolean IN_TESTING = false;
    public static boolean CU_TESTING = false;
    public static int TESTINGID = 0;
    public static double pcoef = 1.0; // Voltage Normalising Term (set during runtime)

    public static boolean coneClaw = false;
    public static boolean armHolding = false;
    public static boolean coneReady = false;

    public static double ep = 0.01;  // Extension pidf and correction term between the motors
    public static double ed = 0;
    public static double ei = 0.0005;
    public static double ef = 0;
    public static double ebp = 0;

    public static double rp = 0.01;  // Lift pidf and correction term between the motors
    public static double rd = 0.0;
    public static double ri = 0.000001;
    public static double rf = 0.0008;
    public static double rbp = 0;

    public static double EAP = 1.0; // Extension A Scale Term
    public static double EBP = 1.0; // Extension B Scale Term
    public static double RAP = 1.0; // Lift A Scale Term
    //public static double RBP = 1.0;
}
