package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.Ports

class IntakeIOReal : IntakeIO {

    override val inputs = LoggedIntakeInput()

    private val centerMotor: SparkMax =
        SparkMax(Ports.Intake.CENTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private val spinMotor: SparkMax = SparkMax(Ports.Intake.SPIN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless)
    private var angleMotor: TalonFX = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val angleControl = PositionVoltage(0.0)


    init {
        angleMotor.configurator.apply(IntakeConstants.MOTOR_CONFIGURATION)
    }

    override fun updateInput() {
        inputs.angle = angleMotor.position.value.times(2 * Math.PI)
        inputs.spinMotorVoltage = Units.Volt.of(spinMotor.busVoltage)
        inputs.angleMotorVoltage = angleMotor.supplyVoltage.value
        inputs.spinMotorVoltage = Units.Volt.of(spinMotor.busVoltage)
    }

    override fun setAngle(angle: Angle) {
        angleMotor.setControl(
            angleControl
                .withPosition(inputs.angle)
        )
    }

    override fun resetAngle() {
        angleMotor.setPosition(0.0)
    }

    override fun setAnglePower(power: Double) {
        angleMotor.set(power)
    }

    override fun setsSpinMotorPower(power: Double) {
        spinMotor.set(power)
    }

    override fun setsCenterMotorPower(power: Double) {
        centerMotor.set(power)
    }

    override fun stopCenterMotor() {
        centerMotor.set(0.0)
    }

    override fun stopSpinMotor() {
        spinMotor.set(0.0)
    }
}