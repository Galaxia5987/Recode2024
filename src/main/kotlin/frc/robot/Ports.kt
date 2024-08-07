package frc.robot

class Ports {
    object Shooter {
        const val TOP_MOTOR_ID: Int = 11
        const val BOTTOM_MOTOR_ID: Int = 10
    }

    object Hood {
        const val MOTOR_ID: Int = 9
        const val ENCODER_ID: Int = 21
    }

    object Conveyor {
        const val MOTOR_ID: Int = 8
    }

    object Gripper {
        const val ANGLE_ID: Int = 6
        const val ROLLER_ID: Int = 7
        const val ENCODER_ID: Int = 7
    }

    object Intake {
        const val ANGLE_ID: Int = 1
        const val ROLLER_ID: Int = 2
        const val CENTER_ID: Int = 3
    }

    object Climb {
        const val MAIN_MOTOR_ID: Int = 4
        const val AUX_MOTOR_ID: Int = 5
        const val STOPPER_ID: Int = 28
    }

    object SwerveDrive {
        const val FRONT_LEFT_DRIVE_MOTOR_ID: Int = 3
        const val FRONT_LEFT_ANGLE_MOTOR_ID: Int = 4
        const val FRONT_RIGHT_DRIVE_MOTOR_ID: Int = 7
        const val FRONT_RIGHT_ANGLE_MOTOR_ID: Int = 8
        const val REAR_LEFT_DRIVE_MOTOR_ID: Int = 5
        const val REAR_LEFT_ANGLE_MOTOR_ID: Int = 2
        const val REAR_RIGHT_DRIVE_MOTOR_ID: Int = 1
        const val REAR_RIGHT_ANGLE_MOTOR_ID: Int = 6

        val DRIVE_IDS: IntArray = intArrayOf(FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                REAR_LEFT_DRIVE_MOTOR_ID,
                REAR_RIGHT_DRIVE_MOTOR_ID
        )
        val ANGLE_IDS: IntArray = intArrayOf(FRONT_LEFT_ANGLE_MOTOR_ID,
                FRONT_RIGHT_ANGLE_MOTOR_ID,
                REAR_LEFT_ANGLE_MOTOR_ID,
                REAR_RIGHT_ANGLE_MOTOR_ID
        )

        const val FRONT_LEFT_ENCODER_ID: Int = 2
        const val FRONT_RIGHT_ENCODER_ID: Int = 7
        const val REAR_LEFT_ENCODER_ID: Int = 3
        const val REAR_RIGHT_ENCODER_ID: Int = 8

        val ENCODER_IDS: IntArray = intArrayOf(FRONT_LEFT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_ID,
                REAR_LEFT_ENCODER_ID,
                REAR_RIGHT_ENCODER_ID
        )

        var FRONT_LEFT_DRIVE_INVERTED: Boolean = true
        var FRONT_LEFT_ANGLE_INVERTED: Boolean = true
        var FRONT_RIGHT_DRIVE_INVERTED: Boolean = true
        var FRONT_RIGHT_ANGLE_INVERTED: Boolean = true
        var REAR_LEFT_DRIVE_INVERTED: Boolean = true
        var REAR_LEFT_ANGLE_INVERTED: Boolean = true
        var REAR_RIGHT_DRIVE_INVERTED: Boolean = true
        val DRIVE_INVERTED: BooleanArray = booleanArrayOf(FRONT_LEFT_DRIVE_INVERTED,
                FRONT_RIGHT_DRIVE_INVERTED,
                REAR_LEFT_DRIVE_INVERTED,
                REAR_RIGHT_DRIVE_INVERTED
        )
        var REAR_RIGHT_ANGLE_INVERTED: Boolean = true
        val ANGLE_INVERTED: BooleanArray = booleanArrayOf(FRONT_LEFT_ANGLE_INVERTED,
                FRONT_RIGHT_ANGLE_INVERTED,
                REAR_LEFT_ANGLE_INVERTED,
                REAR_RIGHT_ANGLE_INVERTED
        )
    }

    object UI {
        const val JOYSTICK_TRIGGER: Int = 1
        const val JOYSTICK_TOP_BOTTOM_BUTTON: Int = 2
        const val JOYSTICK_TOP_LEFT_BUTTON: Int = 3
        const val JOYSTICK_TOP_RIGHT_BUTTON: Int = 4
        const val JOYSTICK_RIGHT_BIG_BUTTON: Int = 16
    }
}
