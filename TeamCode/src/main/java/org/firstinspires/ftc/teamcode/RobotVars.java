package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotVars {
    public static double SMDESCHIS = 0.79; // Claw Very Open (After put)
    public static double SDESCHIS = 0.82; // Claw Open
    public static double SMEDIU = 0.87; // Claw Medium
    public static double SINCHIS = 0.915;   // Claw Close

    public static int EMIN = 7;          // Minimum Extension
    public static int EMAX = 590;         // Maximum Extension

    public static int RTOP_POS = 590;     // Maximum Lift Position
    public static int RMIU_POS = 342;     // Medium Lift Position
    public static int RMID_POS = 100;     // Low Lift Position
    public static int RBOT_POS = 0;       // Bottom Lift position
    public static int LEEW = 0;           // Leeway For Manual Lift Movement

    public static double SAG = 0.400;     // Grabber Arm Get position
    public static double SAH = 0.610;     // Grabber Arm Hold Position
    public static double SAP = 0.74;      // Grabber Arm Put position
    public static double SAW = 0.680;     // Grabber Arm Wait position

    public static double SBAG = 0.59;     // 4bar Get position
    public static double SBAH = 0.39;     // 4bar Hold position
    public static double SBAP = 0.73;     // 4bar Put position

    public static double SDIF = 0.1;      // Grabber Arm Servo Diff Term
    public static double SDIP = 0.0;      // Grabber Arm Servo Proportional Drift Term

    public static double SHG = 0.065;     // Grabber Get Heading
    public static double SHP = 0.50;      // Grabber Put Heading
    public static double SHAP = 0.60;     // Grabber After Put Heading

    public static double SCO = 0.5;       // Cone Securing Mini Servo Open Position
    public static double SCC = 1.0;       // Cone Securing Mini Servo Close Position

    public static double DOT = 0.0;       // Time To Lower The Lift
    public static double UPT = 0.1;       // Time To Hoist Up The Thing
    public static double EXTT = 0.8;      // Time To Fully Extend
    public static double RETT = 0.0;      // Time To Retract

    public static boolean USE_PHOTON = true;

    public static boolean USE_TELE = true; // Use Telemetry
    public static int CU_TESTING = 0;
    public static double pcoef = 1.0; // Voltage Normalising Term (set during runtime)

    public static boolean coneClaw = false;
    public static boolean armHolding = false;
    public static boolean coneReady = false;

    public static double ep = 0.0085;  // Extension pidf and correction term between the motors
    public static double ed = 0;
    public static double ei = 0.0;
    public static double ef = 0;
    public static double emd = 10;
    public static double ebp = 0.0001;

    public static double rp = 0.015;  // Lift pidf and correction term between the motors
    public static double rd = 0.0;
    public static double ri = 0.0;
    public static double rf = 0.001;
    public static double rmd = 10;
    public static double rbp = 0.0018;

    public static double EAP = 1.0; // Extension A Scale Term
    public static double EBP = 1.0; // Extension B Scale Term
    public static double RAP = 1.0; // Lift A Scale Term
    public static double RBP = 1.0;

    public static boolean useExt = true;
    public static boolean useRid = false;

    public static boolean STARTW = true;
    public static String LES = "RF";
    public static String RES = "RB";
    public static String FES = "LF";
    public static boolean LER = true;
    public static boolean RER = true;
    public static boolean FER = true;

    public static boolean AUTO_CLOW = false;

    public static double SHITTY_WORKAROUND_TIME = 0.2;
    public static double SHITTY_WORKAROUND_POWER = 1;

    /*
     * Expansion:
     *     Motors:
     *         0: RB
     *         1: RF
     *         2: LF
     *         3: LB
     *     Servos:
     *         0: sBalans
     *         1:
     *         2: sClose
     *         3:
     *         4: sHeading
     *         5:
     * Control:
     *     Motors:
     *         0: ridB
     *         1: extA
     *         2: ridA
     *         3: extB
     *     Servos:
     *         0: Toate
     *         1: sMCLaw
     *         2:
     *         3: sextA
     *         4:
     *         5: sextB
     *     I2C:
     *         3: csensor
     */
}
