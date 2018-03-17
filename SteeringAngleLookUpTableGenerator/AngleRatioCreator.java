/**
*	<p>
*	ECE 492 Wi18
*	Team 1
*	Part of the Fuzzy Logic Drive Controller Interface
*	</p><p>
*	Creates a C .c/.h file pair containing a 2-Dimensional Array of Integers
*	corresponding to the ideal wheel speed ratios for a vehicle of specified dimensions
*	over a range of incremental angles.
*	Please note that the first ratio is always 512, as it is the ratio of front tire speeds to rear tire speeds.
* 	The second and third ratios, front_left/front_right and rear_left/rear_right, are multipled by 256
*	This makes number processing easier on our system, less time spent in floating point domain.
* 	</p><p>
*	This program compiles using the command "javac AngleRatioCreator.java" command on a Windows 10 machine
*	running Java 8 Update 161
*	The compiled can be executed by running java AngleRatioCreator X U Y Z
*	X - Range of steering angles, e.g. for 45 degrees left or right, it would be 45.
*	U - Increments between angles
*	Y - Length of the vehicles axles (they must be the same length).
*	Z - Width between the two axles. Must be in the units as Y. 
*	The program outputs a .h file with a name given by the input parameters. In the same directory there is
*	a demo file sliplessSteeringRatios.c (made by running the java class file with input args 45 17 30)
*	as well as a C file testCProgram.c. Compile these two using the provided Makefile and then run the output file "./ssr"
*	to see the list of printed numbers (the ratios, multipled by 512 for the first column and 256 for the others).
*	The purpose of the C program is to demostrate that the .c file generated can be inserted into a C program and used without modification.
*	</p><p>
*	Purpose of the file:
*	The file generates the ideal (slipless) speed ratios between the front and rear tires, front left and 
*	front right, and rear left and rear right wheels, given the width and length (Y, Z) over an angle
*	range Z. 
*
*	March 14th Edit
*	This API has been expanded to give the wheel-to-wheel speed ratios between the fastest wheel and the slowest wheel.
*	The C/H file pair is stored in the directory IdealDriveRatios, with a C/H file, a reading C file, a Makefile.
*
*	@since 2018-02-02
*	@author kgmills
*	@version 1.4
*	@param args Three values, see above.
*/

import java.io.*;
import java.lang.Math;
import java.util.ArrayList;

public class AngleRatioCreator {

	private Integer angleRange;
	private Integer increments;
	private Double width;
	private Double length;
	private Integer midPoint;
	private ArrayList<RatioStore> ratioSets;
	private ArrayList<DriveRatioStores> idealSets;
	private	String fileName = "slipLessSteeringRatios.c";
	private String hFileName = "slipLessSteeringRatios.h";
	private String driveCFile = "IdealDriveRatios/motorDriveR.c";
	private String driveHFile = "IdealDriveRatios/motorDriveR.h";
	private Double maxFrontRatio;
	private Double maxRearRatio;
	private Double maxFL;
	private Double maxFR;
	private Double maxRL;
	private Double maxRR;

	public Integer axleRatioMult = 256;
	public Integer overallMult = 512;
	public Float StraightRatio = 1.0000000f;

	/**
	*	Main Function
	* 	Creates an instance of the class, giving it the args, and launches it.
	*	@param args[] Given from command line.
	*/
	public static void main(String[] args) {

		AngleRatioCreator arc = new AngleRatioCreator(args);
	}

	/**
	*	Constructor
	*	Checks input, and if they are right, executes the rest of the code.
	*	@param args[] Given from the main function
	*/
	public AngleRatioCreator(String[] args) {
		if (!this.checkAndSetParams(args)) {
			printRequirements();
			System.exit(0);
		}

		this.midPoint = this.angleRange;
		this.ratioSets = new ArrayList<RatioStore>();
		this.idealSets = new ArrayList<DriveRatioStores>();

		//this.generateLeftTurnRatios();
		this.ratioSets.add(new RatioStore(overallMult, axleRatioMult, axleRatioMult));
		this.idealSets.add(new DriveRatioStores(StraightRatio, StraightRatio, StraightRatio));
		this.generateRightTurnRatios();

		this.writeInfoToCFile();
		this.writeDriveRatiosToFile();
	}

	/** 
	*	Writes information to newly created file, based on input parameters
	*	For Motor Driver
	*/	
	public void writeDriveRatiosToFile() {

		try {
			BufferedWriter headerFile = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(this.driveHFile)));

			headerFile.write("#ifndef motorDriveR_H\n");
			headerFile.write("#define motorDriveR_H\n");
			headerFile.write("extern const float DRIVERATIOS[" + ((this.angleRange/this.increments) + 1) + "][3];\n");
			headerFile.write("#endif");

			headerFile.close();
			System.out.println(this.driveHFile + " created.");

		}

		catch (Exception e) {
			System.err.println(e.getMessage());
		}

		try {
			BufferedWriter ratioFile = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(this.driveCFile)));

			ratioFile.write("#include \"motorDriveR.h\"\n");

			ratioFile.write("const float DRIVERATIOS[" + ((this.angleRange/this.increments) + 1) + "][3] = {\n");

			DriveRatioStores currentTuple;

			for (int i = 0; i < this.idealSets.size(); i++) {
				currentTuple = this.idealSets.get(i);
				String toWrite = "{" + currentTuple.getOpposite() + ", " + currentTuple.getBehind() + ", " 
					+ currentTuple.getDiag() + "}";

				if (i != this.idealSets.size() - 1) {
					toWrite += ",\n";
				}

				else {
					toWrite += "\n};";
				}

				ratioFile.write(toWrite);
			}

			ratioFile.close();
			System.out.println(this.driveCFile + " created.");
		}

		catch (Exception e) {
			System.err.println(e.getMessage());
		}
	}

	/** 
	*	Writes information to newly created file, based on input parameters
	*/	
	public void writeInfoToCFile() {

		try {
			BufferedWriter headerFile = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(this.hFileName)));

			headerFile.write("#ifndef slipLessSteeringRatios_H\n");
			headerFile.write("#define slipLessSteeringRatios_H\n");
			headerFile.write("extern const int SLIPRATIOS[" + ((this.angleRange/this.increments) + 1) + "][3];\n");
			headerFile.write("#endif");

			headerFile.close();
			System.out.println(this.hFileName + " created.");

		}

		catch (Exception e) {
			System.err.println(e.getMessage());
		}

		try {
			BufferedWriter ratioFile = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(this.fileName)));

			ratioFile.write("#include \"slipLessSteeringRatios.h\"\n");

			ratioFile.write("const int SLIPRATIOS[" + ((this.angleRange/this.increments) + 1) + "][3] = {\n");

			RatioStore currentTuple;

			for (int i = 0; i < this.ratioSets.size(); i++) {
				currentTuple = this.ratioSets.get(i);
				String toWrite = "{" + currentTuple.getFR() + ", " + currentTuple.getfLR() + ", " 
					+ currentTuple.getrLR() + "}";

				if (i != this.ratioSets.size() - 1) {
					toWrite += ",\n";
				}

				else {
					toWrite += "\n};";
				}

				ratioFile.write(toWrite);
			}

			ratioFile.close();
			System.out.println(fileName + " created.");
		}

		catch (Exception e) {
			System.err.println(e.getMessage());
		}
	}

	/**
	*	Generates the ratios for left hand turn.
	* 	Not used.
	* 	@deprecated
	*/
	public void generateLeftTurnRatios() {

		Double widthLengthRatio = this.width / this.length;
		Double vfl, vfr, vrl, vrr, frontRatio, rearRatio;
		for(int i = -this.midPoint; i < 0; i = i + this.increments) {
			
			vfl = 1 - (widthLengthRatio*myTan(-i));
			vfr = 1 + (widthLengthRatio*myTan(-i));
			vrl = 1 - ((widthLengthRatio*0.5)*myTan(-i));
			vrr = 1 + ((widthLengthRatio*0.5)*myTan(-i));

			frontRatio = axleRatioMult * (vfl/vfr);
			rearRatio = axleRatioMult * (vrl/vrr);

			this.ratioSets.add(new RatioStore(overallMult, 
				frontRatio.intValue(), rearRatio.intValue()));
		}
	}

	/**
	*	 Generates the ratios for a right hand turn.
	*/
	public void generateRightTurnRatios() {
		
		Double widthLengthRatio = this.width / this.length;
		Double vfl, vfr, vrl, vrr, frontRatio, rearRatio;
		for(int i = this.increments; i <= this.midPoint; i = i + this.increments) {
			
			vfl = 1 + (widthLengthRatio*myTan(i));
			vfr = 1 - (widthLengthRatio*myTan(i));
			vrl = 1 + ((widthLengthRatio*0.5)*myTan(i));
			vrr = 1 - ((widthLengthRatio*0.5)*myTan(i));

			frontRatio = axleRatioMult * (vfl/vfr);
			rearRatio = axleRatioMult * (vrl/vrr);

			if (frontRatio <= 0) {
				frontRatio = this.maxFrontRatio;
			}

			if (rearRatio <= 0) {
				rearRatio = this.maxRearRatio;
			}

			this.ratioSets.add(new RatioStore(overallMult, 
				frontRatio.intValue(), rearRatio.intValue()));

			// Additions for drive task ratios. 

			if (vfr <= 0 && vrr > 0) {
				this.idealSets.add(new DriveRatioStores((float)(this.maxFR/vfl), (float)(vrl/vfl), 
					(float)(vrr/vfl)));
			}

			else if (vfr <= 0 && vrr <= 0) {
				this.idealSets.add(new DriveRatioStores((float)(this.maxFR/vfl), (float)(vrl/vfl), 
					(float)(this.maxRR/vfl)));
			}

			else {
				this.idealSets.add(new DriveRatioStores((float)(vfr/vfl), (float)(vrl/vfl), 
					(float)(vrr/vfl)));
			}
		}
	}

	/**
	*	Checks input parameters for
	*	-A length of three
	*	-The first one being an odd number
	*	-All three of them being convertable to either Integers or Doubles
	*	Stops the program with no ratios made or file if one of these conditions is not met.
	*	@param args[] Input arguments
	*/
	public Boolean checkAndSetParams(String[] args) {

		if (args.length != 4) {
			return false;
		}

		try {
			this.angleRange = Integer.parseInt(args[0]);
			this.increments = Integer.parseInt(args[1]);
			this.width = Double.parseDouble(args[3]);
			this.length = Double.parseDouble(args[2]);
		}

		catch (NumberFormatException e) {
			return false;
		}

		// I added this stuff to compute the angle ratio at the last possible angle
		// Where the numerator is non-zero. 

		Double frontVal = Math.toDegrees(Math.atan(this.length / this.width));
		Double rearVal  = Math.toDegrees(Math.atan((2 * this.length) / this.width));
		Integer maxFrontNonZeroDegree = frontVal.intValue();
		Integer maxRearNonZeroDegree  = rearVal.intValue();

		this.maxFL = 1 + ((this.width * myTan(maxFrontNonZeroDegree)) / this.length);
		this.maxFR = 1 - ((this.width * myTan(maxFrontNonZeroDegree)) / this.length);

		this.maxRL = 1 + ((this.width * myTan(maxRearNonZeroDegree)) / (2 * this.length));
		this.maxRR = 1 - ((this.width * myTan(maxRearNonZeroDegree)) / (2 * this.length));

		this.maxFrontRatio = (maxFL * axleRatioMult) / maxFR;
		this.maxRearRatio = (maxRL * axleRatioMult) / maxRR;

		return true;
	}

	/**
	* A message that is printed in case the requirements of checkAndSetParams is not met.
	*/
	public static void printRequirements() {
		System.out.println("Function takes four numerical arguments:");
		System.out.println("Angle Range: The angle range in one direction.");
		System.out.println("Increments: Spacing between each angle, an integer. E.g. 1 gives you 0, 1 ,2...; 6 gives you 0, 6, 12...");
		System.out.println("Length: Distance between the axles in whatever units you so choose");
		System.out.println("Width: Horizontal distance between the tires, same units as length ");
	}

	/**
	* Custom Math.tan wrapper, with conversion from degrees to radians
	* @param angle The angle in degrees.
	*/
	public Double myTan(Integer angle) {

		return Math.tan(Math.toRadians(angle));
	}


}

/**
*	Separate data-storage class
*	Three data fields, three getters
*/
class RatioStore {

	private Integer frontRear;
	private Integer fLeftRight;
	private Integer rLeftRight;

	public RatioStore(Integer frontRear, Integer fLeftRight, Integer rLeftRight) {
		this.frontRear = frontRear;
		this.fLeftRight = fLeftRight;
		this.rLeftRight = rLeftRight;
	}

	public Integer getFR() {
		return this.frontRear;
	}

	public Integer getfLR() {
		return this.fLeftRight;
	}

	public Integer getrLR() {
		return this.rLeftRight;
	}
}

/**
*	Separate data-storage class
*	For the ratios for normal driving
* 	On a right turn
*	vfl > vrl > vrr > vfr
*/
class DriveRatioStores {

	private Float oppositeWheel;
	private Float behindWheel;
	private Float diagonalWheel;

	public DriveRatioStores(Float oppositeWheel, Float behindWheel, Float diagonalWheel) {
		this.oppositeWheel = oppositeWheel;
		this.behindWheel = behindWheel;
		this.diagonalWheel = diagonalWheel;
	}

	public Float getOpposite() {
		return this.oppositeWheel;
	}

	public Float getBehind() {
		return this.behindWheel;
	}

	public Float getDiag() {
		return this.diagonalWheel;
	}
}