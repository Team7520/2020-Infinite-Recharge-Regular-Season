/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team7520.robot.utilities;

/**
 * Add your docs here.
 */
public class SpeedPlan {

	private double minStartSpeed = 0.1;
	
	private double realCruiseSpeed;
	
	private double maxCruiseSpeed = 0.2;
	
	private double minEndSpeed = 0.08;
	
	private double totalDistance;
	
	private double cruiseDistance;
	
	private double accDistance;
	
	private double decDistance;
	
	private double accAngle = 45;
	
	private double decAngle = 30;
	
	private double outputSpeed;
	
	public void init(double minStartSpeed, double maxCruiseSpeed, double minEndSpeed, 
			double accAngle, double decAngle)
	{
		this.minStartSpeed = minStartSpeed;
		this.maxCruiseSpeed = maxCruiseSpeed;
		this.minEndSpeed = minEndSpeed; 
		this.accAngle = accAngle;
		this.decAngle = decAngle;
	}
	
	public double getSpeed(double distance, double inputCruiseSpeed, double currentPosition)
	{
		totalDistance = distance; 
		
		if(inputCruiseSpeed > maxCruiseSpeed)
		{
			realCruiseSpeed = maxCruiseSpeed;
			//System.out.println("Cruising speed must be at most " + maxCruiseSpeed);	
		}
		else if(inputCruiseSpeed < minStartSpeed)
		{
			realCruiseSpeed = minStartSpeed;
			//System.out.println("Starting speed must be at least " + minStartSpeed);
		}
		
		else 
			realCruiseSpeed = inputCruiseSpeed;
			
		accDistance = (realCruiseSpeed - minStartSpeed) / (Math.tan(Math.toRadians(accAngle)));
		//System.out.println("Acc distance: " + accDistance);
		
		decDistance = (realCruiseSpeed - minEndSpeed) / (Math.tan(Math.toRadians(decAngle)));
		//System.out.println("Dec distance: " + decDistance);
		
		if(totalDistance < accDistance + decDistance)
			cruiseDistance = 0;
		else
			cruiseDistance = totalDistance - (accDistance + decDistance);
		
		//System.out.println("Cruise Distance: " +  cruiseDistance);
		
		if(totalDistance < accDistance)
		{
			outputSpeed = minStartSpeed;
			//System.out.println("Distance is too short, use minStartSpeed only");
		}
		else if(totalDistance < accDistance + decDistance)
		{   // this part needs more detail design  
			outputSpeed = minStartSpeed;
			//System.out.println("Distance is too short, use minStartSpeed only");
		}
		else // normally, distance is long enough for speed up and slow down
		{
		if(currentPosition <= accDistance)
			outputSpeed = ((Math.tan(Math.toRadians(accAngle))) * (currentPosition)) + minStartSpeed; 
			
		else if(currentPosition <= (accDistance + cruiseDistance))
			outputSpeed = realCruiseSpeed;
		
		else if(currentPosition > (accDistance + cruiseDistance) && currentPosition <= totalDistance)
			outputSpeed = (Math.tan(Math.toRadians(decAngle))) * (totalDistance - currentPosition) + minEndSpeed;
		
		}
		return outputSpeed;
		
	}
}

