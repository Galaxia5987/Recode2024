package frc.robot.subsystems.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.ElevatorPorts
import frc.robot.subsystems.hood.GAINS
import frc.robot.subsystems.hood.GEAR_RATIO

class ElevatorIOReal : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val motor = TalonFX(ElevatorPorts.MOTOR_ID)
    private val motorPosititonRequest = PositionVoltage(0.0)
    private val motorPowerRequest = DutyCycleOut(0.0)

        init {
            val motorConfig = TalonFXConfiguration().apply {
                MotorOutput = MotorOutputConfigs().apply {
                    NeutralMode = NeutralModeValue.Brake
                    Inverted = InvertedValue.Clockwise_Positive
                }
                Feedback = FeedbackConfigs().apply {
                    RotorToSensorRatio = 1.0
                    SensorToMechanismRatio = GEAR_RATIO * 0.5
                }
                Slot0 = Slot0Configs().apply {
                    kP = GAINS.kP
                    kI = GAINS.kI
                    kD = GAINS.kD
                }
                CurrentLimits = CurrentLimitsConfigs().apply {
                    StatorCurrentLimitEnable = true
                    SupplyCurrentLimitEnable = true
                    StatorCurrentLimit = 80.0
                    SupplyCurrentLimit = 40.0
                }
            }
        }

    override fun setPosition(position: Double) {
        motor.setControl(motorPosititonRequest.withPosition(position))
    }

    override fun setPower(percentOutput: Double) {
        motor.setControl(motorPowerRequest.withOutput(percentOutput))
    }

    override fun resat() {

    }
    override fun updateInputs() {
        super.updateInputs()
    }
}