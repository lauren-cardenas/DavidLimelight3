/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Map {
     //remote values
        //driving variables
        public static final double TURNING_RATE = 0.85;
        public static final double DRIVING_SPEED = 0.9;
        public static final double SHOOTING_SPEED = 1;

        //controller USB ports
        public static final int DRIVER_CONTROLLER = 0;
        public static final int OPERATOR_CONTROLLER = 1;

        //auto variables
        public static final double AUTO_DISTANCE = 20;
        public static final double AUTO_SPEED = 0.8;

    //motors
        public static final int m_leftMotor1 = 0;
        public static final int m_leftMotor2 = 1;
        public static final int m_rightMotor1 = 2;
        public static final int m_rightMotor2 = 3;

        public static final int m_ShooterLeft = 0;
        public static final int m_ShooterRight = 1;


    //encoder
        public static final int m_leftEnc1 = 0;
        public static final int m_leftEnc2 = 1;
        public static final int m_rightEnc1 = 2;
        public static final int m_rightEnc2 = 3;
        public static final double wheelDiameter = 6;
        public static final double encoderCPR = 360;
}
