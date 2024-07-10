package frc.robot.subsystems.gripper

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import com.revrobotics.SparkLimitSwitch
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer

class GripperIOReal : GripperIO {
    private val rollerMotor: CANSparkMax =
        CANSparkMax(0, CANSparkLowLevel.MotorType.kBrushless) // TODO: Replace deviceID with real value
    private val timer = Timer()
    private val sensor: DigitalInput = DigitalInput(1)

    init {
        rollerMotor.restoreFactoryDefaults()
        rollerMotor.setSmartCurrentLimit(GripperConstants.currentLimit.`in`(Units.Amp).toInt())
        rollerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false)
        rollerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false)
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        rollerMotor.inverted = GripperConstants.ROLLER_INVERTED_VALUE
        rollerMotor.burnFlash()

        timer.start()
        timer.reset()
    }

    override fun setRollerMotorPower(power: Double) {
        rollerMotor.set(power)
    }

    fun hasNote(): Boolean = !sensor.get()

    override fun updateInputs() {
        inputs.rollerMotorVoltage.mut_replace(rollerMotor.get() * RobotController.getBatteryVoltage(), Units.Volts)
        inputs.hasNote = hasNote()
    }
}