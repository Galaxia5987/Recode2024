package frc.robot.subsystems.gripper

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkLimitSwitch
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import frc.robot.Ports

class GripperIOReal : GripperIO {
    override val inputs = LoggedGripperInputs()
    private val rollerMotor: CANSparkMax =
        CANSparkMax(Ports.Gripper.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val timer = Timer()
    private val sensor: DigitalInput = DigitalInput(8)

    init {
        rollerMotor.restoreFactoryDefaults()
        rollerMotor.setSmartCurrentLimit(currentLimit.`in`(Units.Amp).toInt())
        rollerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false)
        rollerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false)
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rollerMotor.inverted = ROLLER_INVERTED_VALUE
        rollerMotor.burnFlash()

        timer.start()
        timer.reset()
    }

    override fun setRollerMotorPower(power: Double) {
        rollerMotor.set(power)
    }

    override fun updateInputs() {
        inputs.rollerMotorVoltage.mut_replace(rollerMotor.busVoltage, Units.Volts)
        inputs.hasNote = !sensor.get()
    }
}