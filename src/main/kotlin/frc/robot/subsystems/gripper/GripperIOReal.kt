package frc.robot.subsystems.gripper

import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.LimitSwitchConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import frc.robot.GripperPorts

class GripperIOReal : GripperIO {
    override val inputs = LoggedGripperInputs()
    private val rollerMotor: SparkMax =
        SparkMax(GripperPorts.ROLLER_ID, SparkLowLevel.MotorType.kBrushless)
    private val timer = Timer()
    private val sensor: DigitalInput = DigitalInput(8)

    init {
        val rollerMotorConfigurator = SparkMaxConfig().apply {
            smartCurrentLimit(CURRENT_LIMIT.`in`(Units.Amp).toInt())
            limitSwitch.apply(LimitSwitchConfig().apply {
                    smartCurrentLimit(CURRENT_LIMIT.`in`(Units.Amp).toInt()) }
            )
            idleMode(SparkBaseConfig.IdleMode.kBrake)
            inverted(ROLLER_INVERTED_VALUE)
        }

        rollerMotor.configure(rollerMotorConfigurator, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        timer.start()
        timer.reset()
    }

    override fun setRollerMotorPower(power: Double) {
        rollerMotor.set(power)
    }

    override fun updateInputs() {
        inputs.rollerMotorVoltage = Units.Volts.of(rollerMotor.busVoltage)
        inputs.hasNote = !sensor.get()
    }
}