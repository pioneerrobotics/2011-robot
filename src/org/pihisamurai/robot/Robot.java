/*
 * Copyright (c) 2011 Team 1076
 */

package org.pihisamurai.robot;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.camera.AxisCamera;

public class Robot extends IterativeRobot {
    
    /***** Configurable variables *****/

    /* cRIO slot configuration */
    final int DIGITAL_IO_SLOT = 4; /* Where the digital IO module is plugged */
    final int ANALOG_IO_SLOT = 1; /* Where the analog IO module is plugged */

    /* Drivetrain jaguar ports */
    final boolean BRAKING = false;

    final int PWM_RIGHT_1 = 4; /* Front right. (River) */
    final int PWM_RIGHT_2 = 1; /* Back right. (Kaylee) */
    final int PWM_LEFT_1 = 3; /* Front left (Inara) */
    final int PWM_LEFT_2 = 2; /* Back left (Zoe) */

    /* For braking, we assume that the Digital I/O for each Jaguar is plugged
       into the same slot and numbers as the PWM plugs. */

    /* Max value drivetrain is capable of */
    final double NOMINALSPEED = 1.0;

    /* Arm victor ports */
    final int ARM_1_1 = 7; /* Elbow 1 */
    final int ARM_1_2 = 10; /* Elbow 2 */
    final int ARM_2 = 8; /* Wrist */
    final int ARM_3 = 9; /* Fingers */

    /* Autonomous configuration */
    final boolean AUTONOMOUS = false; /* Turns autonomous on */
    final long AUTONOMOUS_SAFETY = 5000; /* Time to selfdestruct autonomous */
    final boolean WRIST_NORMAL = true; /* Wrist position flip */
    /* How many milliseconds to lag before autocorrecting */
    final long AUTOCORRECT_LAG = -1;
    /* How many milliseconds to brake when autocorrecting */
    final long AUTOCORRECT_BRAKE = 500;
    /* Fraction of nominal speed we should target in autonomous mode */
    final double AUTONOMOUS_TARGETSPEED = 0.4;

    /* Sensor configuration */
    /* Digital input slots for optical line trackers */
    final int OPT_1 = 9; /* Left */
    final int OPT_2 = 11; /* Right */
    final int OPT_3 = 10; /* Center */

    /* Digital input slots for arm encoders */
    final int ENC_ARM_1 = 7;
    final int ENC_ARM_2 = 12;

    /* Digital input slots for initial limit switches */
    /* Fingers */
    final int SWITCH_FINGER_CLOSED = 12; /* Finger 1 (CLOSED) */
    final int SWITCH_FINGER_OPEN = 13; /* Finger 2 (OPEN) */
    final int SWITCH_FINGER_GRABBED = 1; /* Finger 3 (GRABBED) */

    /* Poll rate for autonomous and teleoperated drivetrain threads in ms */
    final long THREAD_POLL_RATE = 1;

    /* See DriverStationBuffer for camera settings */

    /********************************/

    double PWM_CURRENTSPEED = NOMINALSPEED;

    /* Go for the side position/middle right by default. Directions are from the
       perspective of the robot, not the drivers. */
    boolean middlePosition = false, middleLeft = false;
    boolean wasAutonomous = false;
    boolean wasTeleoperated = false;

    JaguarHelper jr1, jr2, jl1, jl2;

    Drivetrain drivetrain;
    Manipulator manipulator;
    Joystick g27;
    Joystick joystick;

    ToggleListener perspectiveToggle;

    Gyro gyro1;
    DigitalInput opt1, opt2, opt3;
    Encoder enc1, enc2;

    DriverStation driverStation;
    DriverStationBuffer buffer;

    Autonomous autonomousThread;
    Teleoperated teleoperatedThread;

    /* The middle poles on each side are slightly higher than the side pole. Use
       a digital switch on the robot that we will flip prior to the round to set
       it to middle pole mode for autonomous. */
    boolean isMiddlePole;

    public void robotInit() {
        buffer = new DriverStationBuffer();
        driverStation = DriverStation.getInstance();

        buffer.println("*** Robot Init ***");

        drivetrain = new Drivetrain();
        g27 = new Joystick(1);
        joystick = new Joystick(2);

        perspectiveToggle = new ToggleListener(8);

        opt1 = new DigitalInput(DIGITAL_IO_SLOT, OPT_1);
        opt2 = new DigitalInput(DIGITAL_IO_SLOT, OPT_2);
        opt3 = new DigitalInput(DIGITAL_IO_SLOT, OPT_3);
        //enc1 = new Encoder(DIGITAL_IO_SLOT, ENC_1, DIGITAL_IO_SLOT, ENC_1+1);
        //enc2 = new Encoder(DIGITAL_IO_SLOT, ENC_2);

        //enc1.setDistancePerPulse(ENC_DISTANCE_PER_PULSE);
        //enc2.setDistancePerPulse(ENC_DISTANCE_PER_PULSE);
        /*gyro1 = new Gyro(ANALOG_IO_SLOT, GYRO_1);
        gyro1.setSensitivity(GYRO_SENSITIVITY);*/

        manipulator = new Manipulator();

        buffer.println("Init finished");
    }

    public void autonomousInit() {
        killThreads();
        buffer.println("Autonomous");
        wasAutonomous = true;
        if(AUTONOMOUS) {
        buffer.println("AUTO: Auto thread");
        autonomousThread = new Autonomous();
        autonomousThread.start();
        }
        else {
        buffer.println("AUTO: Unavailable");
        }
        buffer.println("AUTO: Init finished");

    }

    public void autonomousPeriodic() {
        //buffer.updateCamera();
    }

    public void disabledInit() {
        killThreads();
    }

    public void teleopInit() {
        killThreads();
        buffer.println("Teleoperated");
        buffer.println("GO GO GO!");
        wasTeleoperated = true;
        buffer.println("Teleop thread");
        teleoperatedThread = new Teleoperated();
        teleoperatedThread.start();
    }

    public void teleopPeriodic() {
        //buffer.updateCamera();
    }
    /* Teleop Continuous: handles arm input */
    public void teleopContinuous() {
        /*if(joystick.getRawButton(11) && joystick.getRawButton(10)) {
            manipulator.setElbowSpeed(0);
        }
        else if(joystick.getRawButton(11)) {
            manipulator.setElbowSpeed(1.0);
        }
        else if(joystick.getRawButton(10)) {
            manipulator.setElbowSpeed(-1.0);
        }
        else {
            manipulator.setElbowSpeed(0);
        }*/

        //Finger movement
        /*if(joystick.getTrigger() && !manipulator.switchFingerGrabbed.get()) {
            manipulator.setFingerSpeed(0);
        }
        else if(joystick.getTrigger() && manipulator.switchFingerOpen.get()) {
            manipulator.setFingerSpeed(1.0);
        }
        else if(!joystick.getTrigger() && manipulator.switchFingerClosed.get())
        {
            manipulator.setFingerSpeed(-1.0);
        }
        else {
            manipulator.setFingerSpeed(0);
        }*/

        if(Math.abs(joystick.getX()) > .2) {

        //buffer.println("switchFO: "+manipulator.switchFingerGrabbed.get());
        manipulator.setFingerSpeed(joystick.getX());
        }
        else {
            manipulator.setFingerSpeed(0);
        }
        
        manipulator.setElbowSpeed(joystick.getY());
        
        //Positive X direction: opening
        //Negative X direction: closing
                        manipulator.setWristSpeed(-joystick.getZ());
        /*
        if(WRIST_NORMAL) {
            if(joystick.getZ() > 0) {
                manipulator.setWristSpeed(-joystick.getZ()*.2);
            }
            else {
                manipulator.setWristSpeed(-joystick.getZ());
            }
        }
        else {
            if(joystick.getZ() < 0) {
                manipulator.setWristSpeed(joystick.getZ());
            }
            else {
                manipulator.setWristSpeed(joystick.getZ()*.2);
            }
        }*/
        //System.out.println(joystick.getZ());
    }

    /* Simple drivetrain class for four motor tank drive */
     private class Drivetrain {

        JaguarHelper r1, r2, l1, l2;

        boolean retard = false;

        Drivetrain() {
            r1 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_RIGHT_1);
            r2 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_RIGHT_2);
            l1 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_LEFT_1);
            l2 = new JaguarHelper(DIGITAL_IO_SLOT, PWM_LEFT_2);
            }

        /* Retard mode true = only runs one pair of motors */
        public boolean setRetardedMode(boolean set) {
            retard = set;
            if(retard) {
                r2.jaguar.set(0);
                l2.jaguar.set(0);
                return true;
            }
            return false;
        }

        /* Sets the speed of the right hand side of the drivetrain */
        public void setRightSpeed(double speed) {
            if(speed > PWM_CURRENTSPEED) {
                speed = PWM_CURRENTSPEED;
            }
            else if(speed < -PWM_CURRENTSPEED) {
                speed = -PWM_CURRENTSPEED;
            }
            r1.jaguar.set(-speed);
            if(!retard) r2.jaguar.set(-speed);
        }

        /* Sets the speed of the left hand side of the drivetrain */
        public void setLeftSpeed(double speed) {
           if(speed > PWM_CURRENTSPEED) {
                speed = PWM_CURRENTSPEED;
            }
            else if(speed < -PWM_CURRENTSPEED) {
                speed = -PWM_CURRENTSPEED;
            }
            l1.jaguar.set(speed);
            if(!retard) l2.jaguar.set(speed);
        }

        public void setBrakeRight(boolean brake) {
            if(BRAKING) {
            r1.digitalOutput.set(brake);
            r2.digitalOutput.set(brake);
            }
        }

        public void setBrakeLeft(boolean brake) {
            if(BRAKING) {
            l1.digitalOutput.set(brake);
            l2.digitalOutput.set(brake);
            }
        }
    }

    /* Class for controlling the 2011 manipulator */
    private class Manipulator {

        /* Limit switches */
        DigitalInput switchFingerClosed, //Fingers: Opened true, else false
                switchFingerOpen, //Fingers: closed true, else false
                switchFingerGrabbed, lim4;

        Victor VICTOR_ELBOW_1, VICTOR_ELBOW_2, VICTOR_WRIST, VICTOR_FINGER;

        /* Encoders */
        Encoder enc1, enc2, enc3;

        /* Victors */
        //Victor VICTOR_ELBOW, VICTOR_ELBOW_2, VICTOR_WRIST, VICTOR_FINGER;

        Manipulator() {
            VICTOR_ELBOW_1 = new Victor(DIGITAL_IO_SLOT, ARM_1_1);
            VICTOR_ELBOW_2 = new Victor(DIGITAL_IO_SLOT, ARM_1_2);
            VICTOR_WRIST = new Victor(DIGITAL_IO_SLOT, ARM_2);
            VICTOR_FINGER = new Victor(DIGITAL_IO_SLOT, ARM_3);

            switchFingerClosed = new DigitalInput(DIGITAL_IO_SLOT,
                    SWITCH_FINGER_CLOSED);
            switchFingerOpen = new DigitalInput(DIGITAL_IO_SLOT,
                    SWITCH_FINGER_OPEN);
            switchFingerGrabbed = new DigitalInput(DIGITAL_IO_SLOT,
                    SWITCH_FINGER_GRABBED);
        }

        void setElbowSpeed(double speed) {
            VICTOR_ELBOW_1.set(speed);
            VICTOR_ELBOW_2.set(speed);
        }

        void setWristSpeed(double speed) {
            VICTOR_WRIST.set(-speed);
            //System.out.println("Victor:" +VICTOR_WRIST.get());
        }

        void setFingerSpeed(double speed) {
            VICTOR_FINGER.set(speed);
        }
    }

    /* Encapsulates the PWM and digital outputs to a single Jaguar. We assume
       that a Jaguar plugged into PWM plug 1 is also connected to Digital IO
       plug 1, and so on. */
    private class JaguarHelper {

        Jaguar jaguar;
        DigitalOutput digitalOutput;

        JaguarHelper(int SLOT, int PLUG) {
            jaguar = new Jaguar(SLOT, PLUG);
            if(BRAKING) {
            digitalOutput = new DigitalOutput(SLOT, PLUG);
            digitalOutput.disablePWM();
            }
        }
    }

    /* Listens to button toggles.
     *
     * Usage: ToggleListener tl = new ToggleListener(buttonNumber). Read boolean
     * from tl.on for current toggle state.
     */
    private class ToggleListener implements Runnable {

        int buttonNumber;
        long pollRate = 10;
        private Thread thread;
        private boolean run = true;
        private boolean isPressed = false;
        public boolean on = false;


    	ToggleListener(int buttonNumber) {
            this.buttonNumber = buttonNumber;
            thread = new Thread(this);
            thread.start();
    	}

    	public void run() {
            while(run) {
                boolean wasPressed = isPressed;

                try {
                if(g27.getRawButton(buttonNumber)) {
                    isPressed = true;
                }
                else {
                    isPressed = false;
                }

                if(wasPressed && !isPressed) {
                    on = !on;
                }

                Thread.sleep(pollRate);
                }
                catch(Exception e) {
                    System.out.println("Exception: "+e);
                }
            }
        }

        void stop() {
            run = false;
        }
        }

    private class Teleoperated implements Runnable {
        final long pollRate = THREAD_POLL_RATE;
        private Thread thread;
        private boolean run = true;

        Teleoperated() {
            thread = new Thread(this);
        }

        void start() {
            thread.start();
        }

        public void run() {
            while(run && driverStation.isOperatorControl()) {
            try {

            int reverse = 1;

            /* Perspective reversal */
            if(perspectiveToggle.on) {
                reverse = -1;
            }

            if(!g27.getRawButton(5) && !g27.getRawButton(6)) {
            /* Positive throttle is brake, negative throttle is accelerate */
            /* Pedals must be on single axis mode */
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            PWM_CURRENTSPEED = 1;
            drivetrain.setRightSpeed(g27.getX()+g27.getThrottle()
                    *reverse);
            drivetrain.setLeftSpeed(-g27.getX()+g27.getThrottle()
                    *reverse);
            }
            else {
            drivetrain.setRightSpeed(0);
            drivetrain.setLeftSpeed(0);
            drivetrain.setBrakeRight(true);
            drivetrain.setBrakeLeft(true);
            }
            Thread.sleep(pollRate);
            }
            catch(Exception e) {
                System.out.println("Exception: "+e);
            }
        }
        }

        void stop() {
            run = false;
            buffer.println("Teleop thread stopped");
        }
    }

    private class Autonomous implements Runnable {

        final long pollRate = THREAD_POLL_RATE;
        /* Cycles we should wait before correcting direction */
        final long autoCorrectLag = AUTOCORRECT_LAG;
        final long autoCorrectBrake = AUTOCORRECT_BRAKE;
        long autoCorrect = 0;
        long autoCorrectStop = 0;
        private Thread thread;
        private boolean run = true;
        DigitalInput sensor1, sensor2, sensor3;
        boolean sensor1On, sensor2On, sensor3On;
        Encoder encoder1;//, encoder2;
        Gyro gyro;
        double gyroPosition = 0;
        double targetSpeed;
        double encoderInitial;

        long safety = AUTONOMOUS_SAFETY;

        public final int PHASE_TRACK = 1;
        public final int PHASE_BRAKE = 2;
        public final int PHASE_SCORE = 3;

    	Autonomous()
        {
            sensor1 = opt1;
            sensor2 = opt2;
            sensor3 = opt3;
            encoder1 = enc1;
            //encoder2 = enc2;
            gyro = gyro1;
            targetSpeed = NOMINALSPEED*AUTONOMOUS_TARGETSPEED;
            thread = new Thread(this);
        }

        /* Only works on straights! */
        double getEncoderDistance() {
             return (encoder1.getDistance());//+encoder2.getDistance())/2;
        }

        void fullForward() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(-targetSpeed);
            drivetrain.setLeftSpeed(-targetSpeed);
        }

        void fullBack() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(targetSpeed);
            drivetrain.setLeftSpeed(targetSpeed);
        }

        void turnRight() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(targetSpeed);
            drivetrain.setLeftSpeed(-targetSpeed);
        }

        void turnLeft() {
            drivetrain.setBrakeRight(false);
            drivetrain.setBrakeLeft(false);
            drivetrain.setRightSpeed(-targetSpeed);
            drivetrain.setLeftSpeed(targetSpeed);
        }

        void fullBrake() {
            drivetrain.setLeftSpeed(0);
            drivetrain.setRightSpeed(0);
            drivetrain.setBrakeLeft(true);
            drivetrain.setBrakeRight(true);
        }

        void start() {
            run = true;
            buffer.println("AUTO: Middle thread start");
            buffer.println("AUTO: Phase 1: Start");
            thread.start();
        }

    	public void run() {

            int phase = PHASE_TRACK;

            while(run && driverStation.isAutonomous()) { //&& safety > 0) {
                try {
                sensor1On = !sensor1.get();
                sensor2On = !sensor2.get();
                sensor3On = !sensor3.get();

                boolean behindLine = true;
                boolean onLine = false;

                /* Phase 1: Robot is on middle line or trying to find middle
                line. Full forward. */
                if(phase == PHASE_TRACK) {
                    buffer.println("AUTO: PHASE_TRACK");
                    if(sensor3On || sensor1On || sensor2On) {
                        safety = AUTONOMOUS_SAFETY;
                    }
                    if(!sensor3On && !sensor1On && !sensor2On) {
                        safety--;
                        autoCorrectStop = 0;
                        fullForward();
                        /* Robot has passed line. Attempt to find line again by
                         going in reverse. */
                        autoCorrect = 0;
                    }

                    else if(sensor3On && !sensor1On && !sensor2On) {
                        autoCorrectStop = 0;
                        onLine = true;
                        fullForward();
                        autoCorrect = 0;
                        /* Take initial gyro reading */
                        if(gyroPosition != 0)
                        gyroPosition = gyro.getAngle();
                    }
                    /* Autocorrect: left sensor ON, right sensor OFF */
                    else if(sensor1On && !sensor2On) {

                        if(!onLine && autoCorrectStop < AUTOCORRECT_BRAKE) {
                            autoCorrectStop++;
                            fullBrake();
                        }
                        else {
                        autoCorrectStop = 0;
                        onLine = true;
                        if(autoCorrect < autoCorrectLag) {
                        fullForward();
                        autoCorrect++;
                        }
                        else {
                            turnLeft();
                        }
                        }
                    }
                     /* Autocorrect: left sensor OFF, right sensor ON */
                    else if(!sensor1On && sensor2On) {

                        if(!onLine && autoCorrectStop < AUTOCORRECT_BRAKE) {
                            autoCorrectStop++;
                            fullBrake();
                        }
                        else {
                        autoCorrectStop = 0;
                        onLine = true;
                        if(autoCorrect < autoCorrectLag) {
                        fullForward();
                        autoCorrect++;
                        }
                        else {
                            turnRight();
                        }
                        }
                    }
                    /* WTF situation: all three sensors on. */
                    else if(sensor3On && sensor1On && sensor2On) {
                        phase++;
                        if(phase == PHASE_SCORE)
                        buffer.println("AUTO: Phase 2: At T");
                    }
                }
                else if(phase == PHASE_SCORE) {
                    /* Turn to match gyro position, score uberring */
                    /* ... */
                }
                Thread.sleep(pollRate);
                }
                catch(Exception e) {
                    System.out.println("Exception: "+e);
                }
            }
            if(safety <= 0) {
            buffer.println("AUTO: Where is line?");
            fullBrake();
            buffer.println("AUTO: U MAD?");
            }

        }

        void stop() {
            run = false;
            buffer.println("AUTO: Thread stopped");
        }
        }

    /* Prints a line to the User Messages box on the driver station software. */
    private class DriverStationBuffer {

        private int CAMERA_MAX_FPS = 5;
        private int CAMERA_COMPRESSION = 0;
        private AxisCamera.ResolutionT CAMERA_RES =
                AxisCamera.ResolutionT.k640x480;

        DriverStationLCD lcd;
        AxisCamera camera;
        String line2 = "", line3 = "", line4 = "", line5 = "", line6 = "";

        /* Null bytes because this library is retarded as fuck */
        final String nullBytes = "\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0";

        DriverStationBuffer() {
            lcd = DriverStationLCD.getInstance();
            //camera = AxisCamera.getInstance();

            //camera.writeResolution(AxisCamera.ResolutionT.k640x480);
            //camera.writeMaxFPS(CAMERA_MAX_FPS);
            //camera.writeCompression(CAMERA_COMPRESSION);
        }

        void println(String line) {
            System.out.println("DRIVERSTATION: "+line);

            line2 = line3;
            line3 = line4;
            line4 = line5;
            line5 = line6;
            line6 = line;

            lcd.println(Line.kUser6, 1, line6+nullBytes);
            lcd.println(Line.kUser5, 1, line5+nullBytes);
            lcd.println(Line.kUser4, 1, line4+nullBytes);
            lcd.println(Line.kUser3, 1, line3+nullBytes);
            lcd.println(Line.kUser2, 1, line2+nullBytes);

            lcd.updateLCD();
        }

        void updateCamera() {
            lcd.updateLCD();
        }
    }

    private void killThreads() {
        if(wasAutonomous) {
            autonomousThread.stop();
        }
        if(wasTeleoperated) {
            teleoperatedThread.stop();
        }

        drivetrain.setRightSpeed(0);
        drivetrain.setLeftSpeed(0);

        wasAutonomous = false;
        wasTeleoperated = false;
    }

    private class analogSwitch {
        AnalogChannel channel;
        analogSwitch(AnalogChannel channel) {
            this.channel = channel;
        }
        boolean get() {
            return (channel.getVoltage() > 2.5);
        }
    }

    /* Special/useful debug functions */
    private void printButtonsHeld() {
        for(int i = 1; i <= 12; i++) {
            boolean held = g27.getRawButton(i);
            if(held)
            System.out.println("Button "+i+": "+held);
        }
    }

    private void printEncoderStatus() {
        System.out.println("en1: "+enc1.get()+" en2: "+enc2.get());
    }

    private void printOpticalStatus() {
        System.out.println("Center: "+opt3.get()+" Left: "+opt1.get()+
            " Right: "+opt2.get());
    }
 }