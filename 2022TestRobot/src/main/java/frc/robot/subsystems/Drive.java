package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
    public enum EncoderType {
        Left,
        Right,
        Average,
        None
    }

    public enum ShifterState {
        Normal,
        Shifted,
        None
    }

    private VictorSP m_leftPrimary;
    private VictorSP m_leftSecondary;
    private VictorSP m_rightPrimary;
    private VictorSP m_rightSecondary;


    private Solenoid m_leftShifter;
    private Solenoid m_rightShifter; 

    private double wheelNonLinearity = .6;
    private double negInertia, oldWheel;
    private double sensitivity;
    private double angularPower;
    private double linearPower;

    private PIDController m_drivePID;
    private PIDController m_rotatePID;

    private ShifterState m_shifterState;

    public Drive() { 
        /*
        Initialize all objects and variables
        Objects are things like motor controllers, solenoids, and sensors
        Variables are things like important numbers/constants to keep track of

        To initialize an object, use general format: ObjectType objectName = new ObjectType(initParameters)
        If you're giving a port number in the object parameters, reference the Constants class: Constants.PortType.STATIC_VALUE
        */
        

        m_leftPrimary = new VictorSP(0);
        m_leftSecondary = new VictorSP(1);
        m_rightPrimary = new VictorSP(2);
        m_rightSecondary = new VictorSP(3);


        m_drivePID = new PIDController(Constants.PID.DRIVE_PROPORTIONAL_COMPETITION, Constants.PID.DRIVE_INTEGRAL_COMPETITION, Constants.PID.DRIVE_DERIVATIVE_COMPETITION);
        m_drivePID.setTolerance(Constants.PID.DRIVE_TOLERANCE_COMPETITION);

        m_rotatePID = new PIDController(Constants.PID.ROTATE_PROPORTIONAL_COMPETITION, Constants.PID.ROTATE_INTEGRAL_COMPETITION, Constants.PID.ROTATE_DERIVATIVE_COMPETITION);
        m_rotatePID.setTolerance(Constants.PID.ROTATE_TOLERANCE_COMPETITION);

        m_shifterState = ShifterState.None;
    }

    @Override
    public void periodic() {
        /*
        A method that loops every 20ms
        Use this to update important values
        Try and avoid using this method; rather, use a command
        Ask mentors before putting things in here!
        */
    }

 

    

    public void cheesyDrive(double throttle, double wheel, double quickTurn, boolean shifted) {
        /*
        This code belongs to the poofs
        Please don't touch this code, but feel free to read and ask questions
        */

        negInertia = wheel - oldWheel;
        oldWheel = wheel;

        wheelNonLinearity = (shifted) ? 0.6 : 0.5;

        /*Apply a sine function that's scaled to make it feel better.*/
        wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);

        sensitivity = (shifted) ? 0.5 : 1.0;

        wheel += negInertia;
        linearPower = throttle;

        angularPower = (quickTurn > 0.5) ? wheel : Math.abs(throttle) * wheel;

        double left = -((linearPower + angularPower) * sensitivity);
        double right = (linearPower - angularPower) * sensitivity;
        driveRaw(left, right);
    }

    public void arcadeDrive(double throttle, double turn) {
        /*Please don't touch this code, but feel free to read and ask questions*/

		double lDrive;
        double rDrive;

        /*reverse turning when driving backwards*/
		if (throttle < -Constants.ARCADE_DRIVE_DEADBAND){
			turn *= -1;
		}

		/*if there is no throttle do a zero point turn, or a "quick turn"*/
		if (Math.abs(throttle) < Constants.QUICK_TURN_DEADBAND) {
			lDrive = turn * Constants.QUICK_TURN_MULTIPLIER;
			rDrive = -turn * Constants.QUICK_TURN_MULTIPLIER;
		} else {
			/*if not driving with quick turn then driveTrain with split arcade*/
			lDrive = throttle * (1 + Math.min(0, turn));
			rDrive = throttle * (1 - Math.max(0, turn));
		}

		driveRaw(lDrive, rDrive);
	}

 



    private void driveRaw(double left, double right) {
	    /*
	    This method is used to supply values to the drive motors
        Variables left and right will be between -1 and 1
        Call the .set(left) for the primary drive motor on the left
        Call the .set(right) for the primary drive motor on the right
	    */
        m_leftPrimary.set(-left);
        m_rightPrimary.set(right);
	}

    public ShifterState getShifterState() {
        /*
        This method is used to get the state of the shifter solenoid
        */
        return m_shifterState;
    }

    public void setShifterState(ShifterState state) {
        /*
        This method changes the state of the solenoid and the value of the member-level variable of type ShifterState 
        to the parameter state
        This method should only be a couple lines!
        */    
        m_shifterState = state;
        // When you add solenoids, make sure to add a line to set them also equal to the shifterState.

        //shifting is inverted. so we invert the set command GH
        if(this.m_shifterState == ShifterState.Normal){
            m_leftShifter.set(false);
            m_rightShifter.set(false);
        }
        else{
            m_leftShifter.set(true);
            m_rightShifter.set(true);
        }

        // old logic that was inverted GH
        // m_leftShifter.set(m_shifterState == ShifterState.Normal);
        // m_rightShifter.set(m_shifterState == ShifterState.Normal);
    }
}
