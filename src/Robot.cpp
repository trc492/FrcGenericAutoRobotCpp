/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This is a generic autonomous program that drives the robot with the specified power for a set amount of time.
 * It works with all 2018 legal motor controllers and 2 or 4-motor drive configured robots.
 * To use this program, you must go through each of the "TODO" sections and select the appropriate configurations
 * and numbers.
 */

//
// TODO: Need to select drive type.
//
#define USE_DIFFERENTIAL_DRIVE
//#define USE_MECANUM_DRIVE

//
// TODO: Need to select motor controller type.
//
//#define USE_DMC60
//#define USE_NIDEC
//#define USE_JAGUAR
//#define USE_SD540
#define USE_SPARK
//#define USE_TALON
//#define USE_TALONSRX
//#define USE_VICTOR
//#define USE_VICTORSP
//#define USE_VICTORSPX

//
// TODO: Need to update the channel numbers.
//       Depending on the motor controller type, these could be PWM channels or CAN IDs.
//
#define LEFT_CHANNEL                            0
#define RIGHT_CHANNEL                           1
#define BRUSHLESS_LEFT_DIO_CHANNEL              0
#define BRUSHLESS_RIGHT_DIO_CHANNEL             1

#define FRONT_LEFT_CHANNEL                      0
#define REAR_LEFT_CHANNEL                       1
#define FRONT_RIGHT_CHANNEL                     2
#define REAR_RIGHT_CHANNEL                      3
#define BRUSHLESS_FRONT_LEFT_DIO_CHANNEL        0
#define BRUSHLESS_REAR_LEFT_DIO_CHANNEL         1
#define BRUSHLESS_FRONT_RIGHT_DIO_CHANNEL       2
#define BRUSHLESS_REAR_RIGHT_DIO_CHANNEL        3

//
// TODO: Need to determine if motors need to be inverted.
//
#define LEFT_MOTOR_INVERTED                     true
#define RIGHT_MOTOR_INVERTED                    false

//
// TODO: Need to tune the following numbers.
//
#define DRIVE_TIME_IN_SEC                       3.0
#define LEFT_DRIVE_POWER                        0.5
#define RIGHT_DRIVE_POWER                       0.5
#define X_DRIVE_POWER                           0.0
#define Y_DRIVE_POWER                           0.5
#define ROTATE_POWER                            0.0

#include <SpeedController.h>

#if defined(USE_DIFFERENTIAL_DRIVE)
  #include <Drive/DifferentialDrive.h>
#elif defined(USE_MECANUM_DRIVE)
  #include <Drive/MecanumDrive.h>
#endif

#if defined(USE_DMC60)
  #include <DMC60.h>
#elif defined(USE_NIDEC)
  #include <NidecBrushless.h>
#elif defined(USE_JAGUAR)
  #include <Jaguar.h>
#elif defined(USE_SD540)
  #include <SD540.h>
#elif defined(USE_SPARK)
  #include <Spark.h>
#elif defined(USE_TALON)
  #include <Talon.h>
#elif defined(USE_TALONSRX)
  #include <ctre/Phoenix.h>
#elif defined(USE_VICTOR)
  #include <Victor.h>
#elif defined(USE_VICTORSP)
  #include <VictorSP.h>
#elif defined(USE_VICTORSPX)
  #include <ctre/Phoenix.h>
#endif

#include <IterativeRobot.h>
#include <Timer.h>

class Robot: public IterativeRobot
{
private:
#if defined(USE_DIFFERENTIAL_DRIVE)
    DifferentialDrive *m_myDifferentialRobot = nullptr;
#elif defined(USE_MECANUM_DRIVE)
    MecanumDrive *m_myMecanumRobot = nullptr;
#endif
    double stopTime;

public:
    /**
     * This method is called one time when the program starts to initialize the robot. It creates and initializes
     * the drive motors and the drive base objects.
     */
    void RobotInit()
    {
#if defined(USE_DIFFERENTIAL_DRIVE)
        SpeedController *leftMotor = nullptr;
        SpeedController *rightMotor = nullptr;

      #if defined(USE_DMC60)
        leftMotor = new DMC60(LEFT_CHANNEL);
        rightMotor = new DMC60(RIGHT_CHANNEL);
      #elif defined(USE_NIDEC)
        leftMotor = new NidecBrushless(LEFT_CHANNEL, BRUSHLESS_LEFT_DIO_CHANNEL);
        rightMotor = new NidecBrushless(RIGHT_CHANNEL, BRUSHLESS_RIGHT_DIO_CHANNEL);
      #elif defined(USE_JAGUAR)
        leftMotor = new Jaguar(LEFT_CHANNEL);
        rightMotor = new Jaguar(RIGHT_CHANNEL);
      #elif defined(USE_SD540)
        leftMotor = new SD540(LEFT_CHANNEL);
        rightMotor = new SD540(RIGHT_CHANNEL);
      #elif defined(USE_SPARK)
        leftMotor = new Spark(LEFT_CHANNEL);
        rightMotor = new Spark(RIGHT_CHANNEL);
      #elif defined(USE_TALON)
        leftMotor = new Talon(LEFT_CHANNEL);
        rightMotor = new Talon(RIGHT_CHANNEL);
      #elif defined(USE_TALONSRX)
        leftMotor = new WPI_TalonSRX(LEFT_CHANNEL);
        rightMotor = new WPI_TalonSRX(RIGHT_CHANNEL);
      #elif defined(USE_VICTOR)
        leftMotor = new Victor(LEFT_CHANNEL);
        rightMotor = new Victor(RIGHT_CHANNEL);
      #elif defined(USE_VICTORSP)
        leftMotor = new VictorSP(LEFT_CHANNEL);
        rightMotor = new VictorSP(RIGHT_CHANNEL);
      #elif defined(USE_VICTORSPX)
        leftMotor = new WPI_VictorSPX(LEFT_CHANNEL);
        rightMotor = new WPI_VictorSPX(RIGHT_CHANNEL);
      #endif
        //
        // If no motor controller type is selected, left and right motors will be null.
        //
        leftMotor->SetInverted(LEFT_MOTOR_INVERTED);
        rightMotor->SetInverted(RIGHT_MOTOR_INVERTED);
        m_myDifferentialRobot = new DifferentialDrive(*leftMotor, *rightMotor);
#elif defined(USE_MECANUM_DRIVE)
        SpeedController *frontLeftMotor = nullptr;
        SpeedController *rearLeftMotor = nullptr;
        SpeedController *frontRightMotor = nullptr;
        SpeedController *rearRightMotor = nullptr;

      #if defined(USE_DMC60)
        frontLeftMotor = new DMC60(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new DMC60(REAR_LEFT_CHANNEL);
        frontRightMotor = new DMC60(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new DMC60(REAR_RIGHT_CHANNEL);
      #elif defined(USE_NIDEC)
        frontLeftMotor = new NidecBrushless(FRONT_LEFT_CHANNEL, BRUSHLESS_FRONT_LEFT_DIO_CHANNEL);
        rearLeftMotor = new NidecBrushless(REAR_LEFT_CHANNEL, BRUSHLESS_REAR_LEFT_DIO_CHANNEL);
        frontRightMotor = new NidecBrushless(FRONT_RIGHT_CHANNEL, BRUSHLESS_FRONT_RIGHT_DIO_CHANNEL);
        rearRightMotor = new NidecBrushless(REAR_RIGHT_CHANNEL, BRUSHLESS_REAR_RIGHT_DIO_CHANNEL);
      #elif defined(USE_JAGUAR)
        frontLeftMotor = new Jaguar(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new Jaguar(REAR_LEFT_CHANNEL);
        frontRightMotor = new Jaguar(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new Jaguar(REAR_RIGHT_CHANNEL);
      #elif defined(USE_SD540)
        frontLeftMotor = new SD540(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new SD540(REAR_LEFT_CHANNEL);
        frontRightMotor = new SD540(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new SD540(REAR_RIGHT_CHANNEL);
      #elif defined(USE_SPARK)
        frontLeftMotor = new Spark(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new Spark(REAR_LEFT_CHANNEL);
        frontRightMotor = new Spark(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new Spark(REAR_RIGHT_CHANNEL);
      #elif defined(USE_TALON)
        frontLeftMotor = new Talon(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new Talon(REAR_LEFT_CHANNEL);
        frontRightMotor = new Talon(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new Talon(REAR_RIGHT_CHANNEL);
      #elif defined(USE_TALONSRX)
        frontLeftMotor = new WPI_TalonSRX(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new WPI_TalonSRX(REAR_LEFT_CHANNEL);
        frontRightMotor = new WPI_TalonSRX(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new WPI_TalonSRX(REAR_RIGHT_CHANNEL);
      #elif defined(USE_VICTOR)
        frontLeftMotor = new Victor(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new Victor(REAR_LEFT_CHANNEL);
        frontRightMotor = new Victor(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new Victor(REAR_RIGHT_CHANNEL);
      #elif defined(USE_VICTORSP)
        frontLeftMotor = new VictorSP(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new VictorSP(REAR_LEFT_CHANNEL);
        frontRightMotor = new VictorSP(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new VictorSP(REAR_RIGHT_CHANNEL);
      #elif defined(USE_VICTORSPX)
        frontLeftMotor = new WPI_VictorSPX(FRONT_LEFT_CHANNEL);
        rearLeftMotor = new WPI_VictorSPX(REAR_LEFT_CHANNEL);
        frontRightMotor = new WPI_VictorSPX(FRONT_RIGHT_CHANNEL);
        rearRightMotor = new WPI_VictorSPX(REAR_RIGHT_CHANNEL);
      #endif
        //
        // If no motor controller type is selected, frontLeft, rearLeft, frontRight and rearRight motors
        // will be null.
        //
        frontLeftMotor->setInverted(LEFT_MOTOR_INVERTED);
        rearLeftMotor->setInverted(LEFT_MOTOR_INVERTED);
        frontRightMotor->setInverted(RIGHT_MOTOR_INVERTED);
        rearRightMotor->setInverted(RIGHT_MOTOR_INVERTED);
        m_myMecanumRobot = new MecanumDrive(
                *frontLeftMotor, *rearLeftMotor, *frontRightMotor, *rearRightMotor);
#endif
    }   // RobotInit

    /**
     * This method is called before autonomous mode starts.
     */
    void AutonomousInit()
    {
        stopTime = Timer::GetFPGATimestamp() + DRIVE_TIME_IN_SEC;
    }   // AutonomousInit

    /**
     * This method is called periodically when autonomous mode is active.
     */
    void AutonomousPeriodic()
    {
        if (Timer::GetFPGATimestamp() < stopTime)
        {
#if defined(USE_DIFFERENTIAL_DRIVE)
            m_myDifferentialRobot->TankDrive(LEFT_DRIVE_POWER, RIGHT_DRIVE_POWER);
#elif defined(USE_MECANUM_DRIVE)
            m_myMecanumRobot->DriveCartesian(X_DRIVE_POWER, Y_DRIVE_POWER, ROTATE_POWER);
#endif
        }
        else
        {
#if defined(USE_DIFFERENTIAL_DRIVE)
            m_myDifferentialRobot->TankDrive(0.0, 0.0);
#elif defined(USE_MECANUM_DRIVE)
            m_myMecanumRobot->DriveCartesian(0.0, 0.0, 0.0);
#endif
        }
    }   // AutonomousPeriodic

};	// class Robot

START_ROBOT_CLASS(Robot)
