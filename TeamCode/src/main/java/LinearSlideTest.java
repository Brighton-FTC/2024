public class LinearSlideTest {

    import
    com.arcrobotics.ftclib.command.InstantCommand;

import
    com.arcrobotics.ftclib.controller.PIDFController;

import
    com.arcrobotics.ftclib.gamepad.GamepadEx;

import
    com.arcrobotics.ftclib.gamepad.GamepadKeys;

import
    com.arcrobotics.ftclib.hardware.ServoEx;

import
    com.arcrobotics.ftclib.hardware.SimpleServo;

import
    com.arcrobotics.ftclib.hardware.motors.Motor;

import
    com.qualcomm.robotcore.eventloop.opmode.Disabled;

import
    com.qualcomm.robotcore.eventloop.opmode.OpMode;

import
    com.qualcomm.robotcore.eventloop.opmode.TeleOp;



    @Disabled

    @TeleOp(name
            =
            "Linear Slide", group =
            "components_test")

    public
    class ArmComponent
            extends
            OpMode {



        /* Motor Declarations */

        private
        Motor LeftRightMotor;

        private
        Motor UpDownMotor;




        /* Position Declarations */

        public
        static final
        int maxLeftPosition
                =
                0;

        public
        static final
        int maxRightPosition
                =
                2000;

        public
        static final
        int maxDownPosition
                =
                0;

        public
        static final
        int maxUpPosition
                = 2000;



        private
        PIDFController pidController;



        private
        final GamepadEx
                gamepad =
                new GamepadEx(gamepad1);



        @Override

        public
        void init() {



// Replace with actual names

            LeftRightMotor
                    =
                    new Motor(hardwareMap,
                            "motorOne");

            UpDownMotor
                    = new
                    Motor(hardwareMap, "motorTwo");



            LeftRightMotor.setRunMode(Motor.RunMode.RawPower);

            UpDownMotor.setRunMode(Motor.RunMode.RawPower);



            pidController
                    =
                    new PIDFController(0,
                            0,
                            0, 0);

            pidController.setTolerance(5);



        }



        @Override

        public
        void loop() {

// Update target positions based on button presses

            updateTargetPositions();



// Use PID controller to move the motors to the target positions

            double
                    power1 =
                    pidController.calculate(LeftRightMotor.getPosition(),
                            LeftRightMotor.getTargetPosition());

            double
                    power2 =
                    pidController.calculate(UpDownMotor.getPosition(),
                            UpDownMotor.getTargetPosition());



// Set power to motors

            LeftRightMotor.setPower(power1);

            UpDownMotor.setPower(power2);



// Display motor positions on telemetry (optional)

            telemetry.addData("Motor1 Position",
                    LeftRightMotor.getPosition());

            telemetry.addData("Motor2 Position",
                    UpDownMotor.getPosition());

            telemetry.update();

        }



        private
        void updateTargetPositions() {

// Update target position for motor 1 based on left and right DPAD

            if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT))
            {

                LeftRightMotor.setTargetPosition(LeftRightMotor.getTargetPosition()
                        -
                        10); // Decrease target position for LeftRightMotor

            } else
            if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT))
            {

                LeftRightMotor.setTargetPosition(LeftRightMotor.getTargetPosition()
                        +
                        10); // Increase target position for LeftRightMotor

            }



// Update target position for motor 2 based on up and down DPAD

            if (gamepad.getButton(GamepadKeys.Button.DPAD_UP))
            {

                UpDownMotor.setTargetPosition(UpDownMotor.getTargetPosition()
                        +
                        10); // Increase target position for UpDownMotor

            } else
            if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN))
            {

                UpDownMotor.setTargetPosition(UpDownMotor.getTargetPosition()
                        -
                        10); // Decrease target position for UpDownMotor

            }

        }

    }

}
