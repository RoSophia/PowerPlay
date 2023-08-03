package org.firstinspires.ftc.teamcode.mk3

import com.qualcomm.robotcore.hardware.AnalogInput

class Lamprey(anin: AnalogInput) {
    private val analIn: AnalogInput

    init {
        analIn = anin
    }

    fun getPosition(): Double {
        return if (analIn.voltage > 3.0) {
            0.0
        } else {
            (analIn.voltage / 3.3) * Math.PI
        }
    }

    fun close() {
        analIn.close()
    }
}