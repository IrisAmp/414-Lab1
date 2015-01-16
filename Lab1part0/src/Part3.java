import java.util.Date;
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
	      {50, 50, 2}
	    };
		
		robot.drive(command);
	}
	
	public static class DeadReckoningRobot
	{
		EncoderMotor motorL;
		EncoderMotor motorR;
		
		private double x = 0.0;
		private double y = 0.0;
		private double theta = 0.0;
		
		public static final double WHEEL_DIAMETER_CM = 5.8;
		public static final double WHEEL_R_CM = 14.6;
		
		public DeadReckoningRobot()
		{
			motorL = new NXTMotor(MotorPort.A);
			motorR = new NXTMotor(MotorPort.B);

			motorL.resetTachoCount();
			motorR.resetTachoCount();
		}
		
		public void drive(int[][] commands)
		{
			Button.waitForAnyPress();
			
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

			System.out.printf("x = %f5\n", x);
			System.out.printf("y = %f5\n", y);
			System.out.printf("w = %f5\n", theta / (2 * Math.PI));
			
			Button.waitForAnyPress();
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
			private double lastTime = (new Date()).getTime();
			
			@Override
			public void run() 
			{
				double currentTime = (new Date()).getTime();
				double dt = currentTime - lastTime;
				lastTime = currentTime;
				
				int tachoL = DeadReckoningRobot.this.motorL.getTachoCount();
				int tachoR = DeadReckoningRobot.this.motorR.getTachoCount();

				motorL.resetTachoCount();
				motorR.resetTachoCount();
				
				double distPerTick = ((DeadReckoningRobot.WHEEL_DIAMETER_CM * Math.PI) / 360.0);
				double deltaDistance = (((double) tachoL + (double) tachoR) / 2.0) * distPerTick;
				double velocity = (deltaDistance / dt);
				
				double ticksPerRot = (DeadReckoningRobot.WHEEL_R_CM * Math.PI) / distPerTick;
				double radPerTick = (2.0 * Math.PI) / ticksPerRot;
				double deltaHeading = ((double) tachoR - (double) tachoL) * (radPerTick / 2.0);
				double rotVelocity = deltaHeading / dt;
				
				DeadReckoningRobot.this.x += velocity * dt * Math.cos(DeadReckoningRobot.this.theta);
				DeadReckoningRobot.this.y += velocity * dt * Math.sin(DeadReckoningRobot.this.theta);;
				DeadReckoningRobot.this.theta += rotVelocity * dt;
			}
		}//*/
	}
}