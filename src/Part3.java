import java.util.Timer;
import java.util.TimerTask;

import lejos.hardware.Button;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;


public class Part3
{
	public static void main(String[] args)
	{
		DeadReckoningRobot robot = new DeadReckoningRobot();
		
		int[][] command = 
			{
	      { 80, 60, 2},
	      { 60, 60, 1},
	      {-50, 80, 2}
	    };
		
		robot.drive(command);
	}
	
	public static class DeadReckoningRobot
	{
		EncoderMotor motorL;
		EncoderMotor motorR;
		
		private float x = 0;
		private float y = 0;
		private float theta = 0;
		
		public static final int WHEEL_DIAMETER_MM = 58;
		public static final int WHEEL_R_MM = 146;
		
		public DeadReckoningRobot()
		{
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);
		}
		
		public void drive(int[][] commands)
		{
			Button.waitForAnyPress();
			
			motorL.resetTachoCount();
			motorR.resetTachoCount();
			
			Timer updater = new Timer();
			updater.schedule(new DeadReckoner(), 0, 10);
			
			for (int[] command : commands)
			{
				motorL.setPower(abs(command[0]));
				motorR.setPower(abs(command[1]));

				if (command[0] < 0)
				{
					motorL.backward();
				}
				else
				{
					motorL.forward();
				}
				
				if (command[1] < 0)
				{
					motorR.backward();
				}
				else
				{
					motorR.forward();
				}
				
				Delay.msDelay(command[2] * 1000);
				
				motorL.stop();
				motorR.stop();
			}
			
			updater.cancel();
			
			System.out.println("x, y, theta");
			System.out.println(x + " " + y + " " + theta);
		}

		private int abs(int i)
		{
			if (i < 0) return -i;
			else return i;
		}
		//*
		private class DeadReckoner
		extends TimerTask
		{
			public static final int TIME_MS = 10;

			@Override
			public void run() 
			{
				int tachoL = DeadReckoningRobot.this.motorL.getTachoCount();
				int tachoR = DeadReckoningRobot.this.motorR.getTachoCount();
				
				double distPerTick = (( (double) DeadReckoningRobot.WHEEL_DIAMETER_MM * Math.PI) / 360.0);
				double deltaDistance = ((tachoL + tachoR) / 2) * distPerTick;
				double velocity = (deltaDistance / TIME_MS);
				
				double ticksPerRot = (DeadReckoningRobot.WHEEL_R_MM * Math.PI) / distPerTick;
				double radPerTick = (2 * Math.PI) / ticksPerRot;
				double deltaHeading = (tachoL - tachoR) * (radPerTick / 2);
				double rotVelocity = deltaHeading / TIME_MS;
				
				DeadReckoningRobot.this.x += velocity * TIME_MS * Math.cos(DeadReckoningRobot.this.theta);
				DeadReckoningRobot.this.y += velocity * TIME_MS * Math.sin(DeadReckoningRobot.this.theta);;
				DeadReckoningRobot.this.theta += rotVelocity * TIME_MS;
			}
		}//*/
	}
}
