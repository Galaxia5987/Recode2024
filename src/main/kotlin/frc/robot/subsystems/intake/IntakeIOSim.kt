package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.SparkSim
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import frc.robot.Ports
import frc.robot.lib.motors.SparkMaxSim
import frc.robot.lib.motors.TalonFXSim

class IntakeIOSim : IntakeIO {
    override val inputs: LoggedIntakeInput = LoggedIntakeInput()
    val centerMotor = SparkMaxSim(1, 1.0, IntakeConstants.MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters), 1.0)
    val angleMotor = TalonFXSim(1, 1.0, IntakeConstants.MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters), 1.0)
    val spinMotor = TalonFXSim(1, IntakeConstants.GEAR_RATIO, 1.0, 360 * IntakeConstants.GEAR_RATIO)
    val anglePIDController: PIDController =
        PIDController(IntakeConstants.ANGLE_KP, IntakeConstants.ANGLE_KI, IntakeConstants.ANGLE_KD)
    private val angleControl = PositionVoltage(0.0)
    private val dutyCycle = DutyCycleOut(0.0)


    init {
        angleMotor.setController(anglePIDController)
    }

    override fun updateInput() {
        inputs.angle = Units.Rotations.of(angleMotor.position * 2 * Math.PI)
        inputs.angleMotorVoltage = Units.Volt.of(angleMotor.appliedVoltage)
        inputs.spinMotorVoltage = Units.Volt.of(spinMotor.appliedVoltage)
        inputs.centerMotorVoltage = Units.Volt.of(centerMotor.busVoltage)
    }

    override fun setAngle(angle: Angle) {
        angleMotor.setControl(angleControl.withPosition(angle))
    }

    override fun resetAngle() {
        angleMotor.setControl(angleControl.withPosition(0.0))

    }

    override fun setAnglePower(power: Double) {
        angleMotor.setControl(dutyCycle.withOutput(power))
    }

    override fun setsSpinMotorPower(power: Double) {
        spinMotor.setControl(DutyCycleOut(power))
    }

    override fun setsCenterMotorPower(power: Double) {
        centerMotor.set(power)
    }

    override fun stopCenterMotor() {
        centerMotor.set(0.0)
    }

    override fun stopSpinMotor() {
        spinMotor.setControl(DutyCycleOut(0.0))
    }
}