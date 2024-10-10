package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class ShillingTank{

    AHRS m_navx;
    CANSparkMax m_rightNeoMaster;
    CANSparkMax m_rightNeoSlave;
    RelativeEncoder m_rightEncoder;
    CANSparkMax m_leftNeoMaster;
    CANSparkMax m_leftNeoSlave;
    RelativeEncoder m_leftEncoder;

    double box1 = 0;
    double box2 = 0;
    double box3 = 0;
    double box4 = 0;

    double coefficent = 1;
    double pCoeffient = 1;

    public ShillingTank(
        int leftA,
        int leftB,
        int rightA,
        int rightB,
        boolean rightInvert,
        boolean leftInvert,
        int maxCurrent,
        AHRS gyro
    ){
        m_navx = gyro;
    
        m_rightNeoMaster.restoreFactoryDefaults();
        m_leftNeoMaster.restoreFactoryDefaults();
        m_rightNeoSlave.restoreFactoryDefaults();
        m_leftNeoSlave.restoreFactoryDefaults();

        m_rightNeoMaster.setInverted(rightInvert);
        m_rightNeoSlave.setInverted(rightInvert);
        m_leftNeoMaster.setInverted(leftInvert);
        m_leftNeoSlave.setInverted(leftInvert);

        m_rightNeoSlave.follow(m_rightNeoMaster);
        m_leftNeoSlave.follow(m_leftNeoMaster);

        m_rightNeoMaster.setSmartCurrentLimit(maxCurrent, maxCurrent);
        m_rightNeoSlave.setSmartCurrentLimit(maxCurrent, maxCurrent);
        m_leftNeoMaster.setSmartCurrentLimit(maxCurrent, maxCurrent);
        m_leftNeoSlave.setSmartCurrentLimit(maxCurrent, maxCurrent);

        m_rightNeoMaster = new CANSparkMax(rightA, MotorType.kBrushless);
        m_rightNeoSlave = new CANSparkMax(rightB, MotorType.kBrushless);
        m_rightEncoder = m_rightNeoMaster.getEncoder();

        m_leftNeoMaster = new CANSparkMax(leftA, MotorType.kBrushless);
        m_leftNeoSlave = new CANSparkMax(leftB, MotorType.kBrushless);
        m_leftEncoder = m_leftNeoMaster.getEncoder();
        
    }

    public void resetEncoders(){
        m_rightEncoder.setPosition(0);
        m_leftEncoder.setPosition(0);
    }

    public void setIdleMode(IdleMode mode){
        m_rightNeoMaster.setIdleMode(mode);
        m_rightNeoSlave.setIdleMode(mode);
        m_leftNeoMaster.setIdleMode(mode);
        m_leftNeoSlave.setIdleMode(mode);
    }

    double outputT = 0;
    double outputD = 0;
    double ramp = 0.2;

    public void arcadeDrive(double speed, double turn){
        speed = speed * coefficent;
    
        turn = deadband(turn/2);
        speed = deadband(speed);

        outputD = outputD + (outputD - speed) * -ramp;
        outputT = outputT + (outputT - turn) * -ramp * 1.1;
        
        m_rightNeoMaster.set(outputD+turn);
        m_leftNeoMaster.set(outputD-turn);

    }

    double deadband(double value) {
        /* Upper deadband */
        if (value >= +0.10 ) 
            return value-0.1;
        
        /* Lower deadband */
        if (value <= -0.10)
            return value+0.1;
        
        /* Outside deadband */
        return 0;
    }

    public void setCoefficient(double c){ //speed modifier
        coefficent = c;
    }

    public void prevCoeffient(double prevC){ //stores previous speed
        prevC = pCoeffient;
    }

    public double getLeftPosition(){
        return m_leftEncoder.getPosition();
    }

    public double getRightPosition(){
        return m_rightEncoder.getPosition();
    }
}