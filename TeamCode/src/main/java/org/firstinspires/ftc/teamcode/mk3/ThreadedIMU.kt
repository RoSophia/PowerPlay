package org.firstinspires.ftc.teamcode.mk3

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import java.lang.Thread.sleep

class ThreadedIMU(im: BNO055IMU) : Runnable {
    private val imu: BNO055IMU
    var lastRead: Double = 0.0
    var isRunning = true
    lateinit var lom: LinearOpMode

    init {
        imu = im
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.mode = BNO055IMU.SensorMode.IMU
        imu.initialize(parameters)
    }

    fun close() {
        imu.close();
    }

    override fun run() {
        val ET = ElapsedTime()
        while (Thread.currentThread().isAlive && isRunning && !lom.isStopRequested) {
            ET.reset()
            val ar = imu.angularOrientation.firstAngle.toDouble()
            lastRead = (ar + Math.PI * 2) % (2 * Math.PI)
            val tp = TelemetryPacket()
            tp.put("IMU_CYCLE_TIME", ET.seconds())
            tp.put("IMU_ACTUAL_READ", ar)
            tp.put("IMU_FIXEED_READ", lastRead)
            FtcDashboard.getInstance().sendTelemetryPacket(tp)
            sleep(3)
        }
    }
}