package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDs private constructor(port: Int, length: Int) : SubsystemBase() {
    private val ledStrip = AddressableLED(port)
    private val ledBuffer = AddressableLEDBuffer(length)
    private var mode = Mode.SOLID
    private val timer = Timer()
    private val intakeTimer = Timer()
    private var blinkTime = 0.5
    private var rainbowFirstPixelHue = 0

    companion object {
        @Volatile
        private var INSTANCE: LEDs? = null

        fun initialize(port: Int, length: Int) {
            synchronized(this) {
                if (INSTANCE == null) {
                    INSTANCE = LEDs(port, length)
                }
            }
        }

        fun getInstance(): LEDs {
            return INSTANCE ?: throw IllegalStateException("LEDs have not been initialized")
        }
    }

    /**
     * The primary color of the LEDs that will be used in the solid and percentage
     * modes and as the first color in the blink and fade modes.
     */
    private var primary = Color.kBlack

    /**
     * The secondary color of the LEDs that will be used as the second color in
     * the blink and fade modes.
     */
    private var secondary = Color.kBlack

    init {
        ledStrip.setLength(length)
        ledStrip.setData(ledBuffer)
        ledStrip.start()

        timer.start()
        timer.reset()
        intakeTimer.start()
        intakeTimer.reset()
    }

    fun setSolidMode(color: Color): Command {
        return Commands.runOnce({
            primary = color
            mode = Mode.SOLID
        })
    }

    fun setBlink(primary: Color, secondary: Color, blinkTime: Double): Command {
        return Commands.runOnce({
            this.primary = primary
            this.secondary = secondary
            this.blinkTime = blinkTime
            mode = Mode.BLINK
        })
    }

    fun setRainbow(): Command {
        return Commands.runOnce({
            mode = Mode.RAINBOW
        })
    }

    fun intakeCommand(): Command {
        return Commands.runOnce({
            val currentPrimary = primary
            val currentSecondary = secondary
            val currentBlinkTime = blinkTime
            val currentMode = mode
            setBlink(Color.kBlack, LEDConstants.HAS_NOTE_COLOR, 0.1)
                .andThen(Commands.waitSeconds(2.0))
                .andThen(Commands.runOnce(
                    {
                        when (currentMode) {
                            Mode.SOLID -> setSolidMode(currentPrimary)
                            Mode.BLINK -> setBlink(currentPrimary, currentSecondary, currentBlinkTime)
                            Mode.RAINBOW -> setRainbow()
                        }
                    }
                ))
        })
    }

    private fun setSolidColor(color: Color) {
        val dimmed = Color(color.red*0.2,color.green*0.2,color.blue*0.2)
        for (i in 0 until ledBuffer.length) {
            ledBuffer.setLED(i, dimmed)
        }
        ledStrip.setData(ledBuffer)
    }

    private enum class Mode {
        SOLID,
        BLINK,
        RAINBOW
    }

    override fun periodic() {
        when (mode) {
            Mode.SOLID -> {
                setSolidColor(primary)
            }

            Mode.BLINK -> {
                setSolidColor(primary)
                timer.advanceIfElapsed(blinkTime)
                setSolidColor(secondary)
                timer.advanceIfElapsed(blinkTime)
            }

            Mode.RAINBOW -> {
                if (timer.advanceIfElapsed(0.05)) {
                    for (i in 0 until ledBuffer.length) {
                        // Calculate the hue - hue is easier for rainbows because the color
                        // shape is a circle so only one value needs to precess

                        val hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.length)) % 180

                        // Set the value
                        ledBuffer.setHSV(i, hue, 255, 128)
                    }
                    rainbowFirstPixelHue += 8
                    // Check bounds
                    rainbowFirstPixelHue %= 180
                    ledStrip.setData(ledBuffer)
                }
                // Increase by to make the rainbow "move"
            }
        }
    }
}
