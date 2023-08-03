package org.firstinspires.ftc.teamcode.mk3.camera

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.BC
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.CAMERA_UPDATE
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.COL_INDEX
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.CUR_DONE_CORRECTION
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.DO_I_EVEN_PROCESS_FRAME
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.DRAW_BOXES
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.DRAW_MEDIAN
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.INVERT
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.RC
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.TB
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.TR
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.CameraControls.USE_TELE
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs
import kotlin.math.sqrt

class Box2d(x: Int, y: Int, w: Int, h: Int) : Comparable<Box2d> {
    var sx: Int = 0
    var sy: Int = 0
    var width: Int = 0
    var height: Int = 0

    constructor() : this(0, 0, 0, 0)

    init {
        sx = x
        sy = y
        width = w
        height = h
    }

    fun gdist(): Double {
        val xd = sx.toDouble() + width.toDouble() / 2
        val yd = sy.toDouble() + height.toDouble() / 2
        return sqrt(xd * xd + yd * yd)
    }

    override fun compareTo(other: Box2d): Int {
        return if (gdist() < other.gdist()) {
            1
        } else {
            -1
        }
    }
}

class ConePipeline(height: Int, width: Int) : OpenCvPipeline() {
    var xoff: Double = 0.0

    var results: Array<Box2d> = arrayOf(Box2d())

    private fun mklocations(up: Int, left: Int, width: Int, xoff: Int, yoff: Int, h: Int, w: Int): Array<Box2d> {
        val len: Int = (up * 2 - 1) * (left * 2 - 1)
        val res = Array(len) { Box2d() }
        var ci = 0
        for (i in -(up - 1) until up) {
            for (j in -(left - 1) until left) {
                res[ci] = Box2d(i * width + h / 2 - width / 2 + xoff, j * width + w / 2 - width / 2 + yoff, width, width)
                ++ci
            }
        }

        results.sort()
        return res
    }

    private fun draw(frame: Mat, cb: Box2d, vl: Double, ll: Double) {
        val p1 = Point(cb.sx.toDouble(),
                cb.sy.toDouble())
        val p3 = Point(cb.sx.toDouble() + cb.width.toDouble(),
                cb.sy.toDouble() + cb.height.toDouble())

        val red = Scalar(ll, ll, ll, CameraControls.ALPHA)
        val col = Scalar(vl, vl, vl, CameraControls.ALPHA)

        Imgproc.rectangle(frame, p1, p3, red, 7)
        Imgproc.rectangle(frame, p1, p3, col, -1)
    }

    @Config
    object CameraControls {
        @JvmField
        var WIDTH: Int = 15

        @JvmField
        var LUP: Int = 9

        @JvmField
        var LUT: Int = 18

        @JvmField
        var XOFF: Int = 0

        @JvmField
        var YOFF: Int = 100

        @JvmField
        var INVERT: Boolean = false

        @JvmField
        var ALPHA: Double = 200.0

        @JvmField
        var TB: Double = 75.0

        @JvmField
        var TR: Double = 25.0

        @JvmField
        var COL_INDEX: Int = 1

        @JvmField
        var TARGET_BRIGHTNESS: Double = 1.0

        var RC: Double = 0.5
        var BC: Double = 1.0

        @JvmField
        var TP: Double = 1.0

        @JvmField
        var DRAW_BOXES: Boolean = false

        @JvmField
        var DRAW_MEDIAN: Boolean = true

        @JvmField
        var USE_TELE: Boolean = false

        @JvmField
        var DO_I_EVEN_PROCESS_FRAME: Boolean = true

        @JvmField
        var CAMERA_UPDATE: Boolean = false

        @JvmField
        var CUR_DONE_CORRECTION: Int = 0
    }

    private fun subm(img: Mat, box: Box2d): Mat {
        return img.submat(box.sy, box.sy + box.height, box.sx, box.sx + box.width)
    }

    private fun check(img: Mat, box: Box2d): DoubleArray {
        val subm = subm(img, box)
        return Core.mean(subm).`val`
    }

    private fun isRed(col: DoubleArray): Boolean {
        val b = col[0]
        val g = col[1]
        val r = col[2]
        val rcoef = (-(r / 255) + 1.5) * RC
        val bg = b - g
        val bcoef = (-(b / 255) + 1.5) * BC

        /*
        if (USE_TELE) {
            val tp = TelemetryPacket()
            tp.put("abs(r - g)", abs(r - g))
            tp.put("rcoef", rcoef)
            tp.put("abs(r - g) * rcoef", abs(r - g) * rcoef)

            tp.put("bg", bg)
            tp.put("bcoef", bcoef)
            tp.put("bg * bcoef", bg * bcoef)

            FtcDashboard.getInstance().sendTelemetryPacket(tp)
        }*/

        if (abs(r - g) * rcoef < TR) {
            if (bg > TB) {
                return true
            }
        }
        return false
    }

    var checkLocations: Array<Box2d> = if (INVERT) {
        mklocations(CameraControls.LUP, CameraControls.LUT, CameraControls.WIDTH, CameraControls.YOFF, CameraControls.XOFF, height, width)
    } else {
        mklocations(CameraControls.LUT, CameraControls.LUP, CameraControls.WIDTH, CameraControls.XOFF, CameraControls.YOFF, width, height)
    }

    private val ourTimer = ElapsedTime(0)
    private val theirTimer = ElapsedTime(0)
    private var startTimer = false
    override fun processFrame(input: Mat): Mat {
        if (!startTimer) {
            theirTimer.reset()
            startTimer = true
        }
        ourTimer.reset()
        if (input.empty()) {
            return input
        }
        if (DO_I_EVEN_PROCESS_FRAME) {
            val frame = Mat()
            input.copyTo(frame)


            val ff = Mat()
            if (DRAW_BOXES || DRAW_MEDIAN) {
                frame.copyTo(ff)
            }
            var medXS = 0
            var medYS = 0
            var redc = 0
            //val tp = TelemetryPacket()
            for ((ci, element) in checkLocations.withIndex()) {
                val vl = check(frame, element)
                /*
                if (checkLocations.size <= 6 && USE_TELE) {
                    for (i in vl.indices) {
                        tp.put("val$ci _$i", vl[i])
                    }
                }*/
                if (isRed(vl)) {
                    medXS += element.sx + element.width / 2
                    medYS += element.sy + element.height / 2
                    ++redc
                    if (DRAW_BOXES) {
                        draw(ff, element, 255.0, vl[COL_INDEX])
                    }
                } else {
                    if (DRAW_BOXES) {
                        draw(ff, element, 0.0, vl[COL_INDEX])
                    }
                }
            }
            /*
            if (checkLocations.size <= 6 && USE_TELE) {
                FtcDashboard.getInstance().sendTelemetryPacket(tp)
            }*/

            val w = frame.width()
            val h = frame.height()
            val medX = if (redc > 0) {
                medXS.toDouble() / redc.toDouble()
            } else {
                w / 2.0
            }
            val medY = if (redc > 0) {
                medYS.toDouble() / redc.toDouble()
            } else {
                h / 2.0
            }

            if (DRAW_MEDIAN && !ff.empty()) {
                val p1 = Point(w / 2.0, h / 2.0)
                val p2 = Point(medX, medY)

                val c1 = Point(w / 2.0 + CUR_DONE_CORRECTION * 20, 10.0)
                val c2 = Point(w / 2.0, 10.0)

                Imgproc.line(ff, c1, c2, Scalar(0.0, 0.0, 0.0, 255.0), 9)
                if (CAMERA_UPDATE) {
                    Imgproc.line(ff, c1, c2, Scalar(0.0, 255.0, 0.0, 255.0), 8)
                } else {
                    Imgproc.line(ff, c1, c2, Scalar(200.0, 0.0, 255.0, 255.0), 8)
                }

                if (CAMERA_UPDATE) {
                    Imgproc.line(ff, p1, p2, Scalar(0.0, 0.0, 0.0, 255.0), 6)
                    Imgproc.line(ff, p1, p2, Scalar(255.0, 0.0, 0.0, 255.0), 4)
                } else {
                    Imgproc.line(ff, p1, p2, Scalar(255.0, 255.0, 255.0, 255.0), 6)
                }
            }

            xoff = medX - (w / 2.0)
            //xoff = ((medX - (w / 2.0)) / w) * 2

            if (USE_TELE) {
                val telp = TelemetryPacket()
                telp.put("GOT XOFF", xoff)
                telp.put("CONE_PIPELINE_MY_CYCLE", ourTimer.seconds())
                telp.put("CONE_PIPELINE_THEIR_CYCLE", theirTimer.seconds())
                ourTimer.reset()
                theirTimer.reset()
                FtcDashboard.getInstance().sendTelemetryPacket(telp)
            }

            return if ((DRAW_BOXES || DRAW_MEDIAN) && !ff.empty()) {
                ff
            } else {
                frame
            }
        } else {
            if (USE_TELE) {
                val telp = TelemetryPacket()
                telp.put("GOT XOFF", xoff)
                telp.put("CONE_PIPELINE_MY_CYCLE", ourTimer.seconds())
                telp.put("CONE_PIPELINE_THEIR_CYCLE", theirTimer.seconds())
                ourTimer.reset()
                theirTimer.reset()
                FtcDashboard.getInstance().sendTelemetryPacket(telp)
            }
            return input
        }
    }
}
