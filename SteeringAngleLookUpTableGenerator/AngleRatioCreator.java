/**
*	<p>
*	ECE 492 Wi18
*	Team 1
*	Part of the Fuzzy Logic Drive Controller Interface
*	</p><p>
*	Creates a C header file containing a 2-Dimensional Array of Integers
*	corresponding to the ideal wheel speed ratios for a vehicle of specified dimensions
*	over a range of incremental angles.
*	Please note that the first ratio is always 512, as it is the ratio of front tire speeds to rear tire speeds.
* 	The second and third ratios, front_left/front_right and rear_left/rear_right, are multipled by 256
*	This is so our Microcontroller board can do these comparrisons with integers and not floating point numbers.
* 	</p><p>
*	This program compiles using the command "javac AngleRatioCreator.java" command on a Windows 10 machine
*	running Java 8 Update 161
*	The compiled can be executed by running java AngleRatioCreator X Y Z
*	X - Range of steering angles, e.g. for 45 degrees left to right, this number would be 91
*	For 45 in the left, 1 in the center, 45 in the right. This number HAS to be odd 
*	or the program will reject it.
*	Y - Width of the vehicles axles (they must be the same length).
*	Z - Length between the two axles. Must be in the units as Y. 
*	The program outputs a .h file with a name given by the input parameters. In the same directory there is
*	a demo file sliplessSteeringRatios_33R_3W_10L.h (made by running the java class file with input args 33 3 10)
*	as well as a C file testCProgram.c. Compile these two using 
*	"gcc testCProgram.c sliplessSteeringRatios_33R_3W_10L.h -o ssr -std=c99 -Wall" and then run "./a.out"
*	to see the list of printed numbers (the ratios, multipled by 512 for the first column and 256 for the others).
*	The purpose of the C program is to demostrate that the .h file generated can be inserted into a C program and used without modification.
*	</p><p>
*	Purpose of the file:
*	The file generates the ideal (slipless) speed ratios between the front and rear tires, front left and 
*	front right, and rear left and rear right wheels, given the width and length (Y, Z) over an angle
*	range Z. 
*
*	@since 2018-02-02
*	@author kgmills
*	@version 1.0
*	@param args Three values, see above.
*/

import java.io.*;
import java.lang.Math;
import java.util.ArrayList;

public class AngleRatioCreator {

	private Integer angleRange;
	private Double width;
	private Double length;
	private Integer midPoint;
	private ArrayList<RatioStore> ratioSets;

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

		this.midPoint = (this.angleRange - 1) / 2;
		this.ratioSets = new ArrayList<RatioStore>();

		this.generateLeftTurnRatios();
		this.ratioSets.add(new RatioStore(512, 256, 256));
		this.generateRightTurnRatios();

		this.writeInfoToCFile();
	}

	/** 
	*	Writes information to newly created file, based on input parameters
	*/	
	public void writeInfoToCFile() {

		String fileName = "sliplessSteeringRatios_" + 
			this.angleRange + "R_" + this.width.intValue() + "W_" 
			+ this.length.intValue() + "L.h";

		try {
			BufferedWriter ratioFile = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(fileName)));

			ratioFile.write("#ifndef " + fileName.replace(".h", "_H") + "\n");
			ratioFile.write("#define " + fileName.replace(".h", "_H") + "\n");

			ratioFile.write("const int SLIPRATIOS[" + this.angleRange + "][3] = {\n");

			RatioStore currentTuple;

			for (int i = 0; i < this.ratioSets.size(); i++) {
				currentTuple = this.ratioSets.get(i);
				String toWrite = "{" + currentTuple.getFR() + ", " + currentTuple.getfLR() + ", " 
					+ currentTuple.getrLR() + "}";

				if (i != this.ratioSets.size() - 1) {
					toWrite += ",\n";
				}

				else {
					toWrite += "\n};\n#endif";
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
	*/
	public void generateLeftTurnRatios() {

		Double widthLengthRatio = this.width / this.length;
		Double vfl, vfr, vrl, vrr, frontRatio, rearRatio;
		for(int i = -this.midPoint; i < 0; i++) {
			
			vfl = 1 - (widthLengthRatio*myTan(-i));
			vfr = 1 + (widthLengthRatio*myTan(-i));
			vrl = 1 - ((widthLengthRatio*0.5)*myTan(-i));
			vrr = 1 + ((widthLengthRatio*0.5)*myTan(-i));

			frontRatio = 256 * (vfl/vfr);
			rearRatio = 256 * (vrl/vrr);

			this.ratioSets.add(new RatioStore(512, 
				frontRatio.intValue(), rearRatio.intValue()));
		}
	}

	/**
	*	 Generates the ratios for a right hand turn.
	*/
	public void generateRightTurnRatios() {
		
		Double widthLengthRatio = this.width / this.length;
		Double vfl, vfr, vrl, vrr, frontRatio, rearRatio;
		for(int i = 1; i <= this.midPoint; i++) {
			
			vfl = 1 + (widthLengthRatio*myTan(i));
			vfr = 1 - (widthLengthRatio*myTan(i));
			vrl = 1 + ((widthLengthRatio*0.5)*myTan(i));
			vrr = 1 - ((widthLengthRatio*0.5)*myTan(i));

			frontRatio = 256 * (vfl/vfr);
			rearRatio = 256 * (vrl/vrr);

			this.ratioSets.add(new RatioStore(512, 
				frontRatio.intValue(), rearRatio.intValue()));
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

		if (args.length != 3) {
			return false;
		}

		try {
			this.angleRange = Integer.parseInt(args[0]);
			this.width = Double.parseDouble(args[1]);
			this.length = Double.parseDouble(args[2]);

			if ((angleRange % 2) != 1) {
				return false;
			}
		}

		catch (NumberFormatException e) {
			return false;
		}

		return true;
	}

	/**
	* A message that is printed in case the requirements of checkAndSetParams is not met.
	*/
	public static void printRequirements() {
		System.out.println("Function takes three numerical arguments:");
		System.out.println("Angle Range: The angle range of steering movement, an odd degree number");
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