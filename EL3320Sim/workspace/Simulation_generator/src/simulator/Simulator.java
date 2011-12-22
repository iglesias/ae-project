package simulator;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Random;

import simulator.Position;


public class Simulator {
	
	public ArrayList<Position> readFile(String fileName) {
		ArrayList<Position> positions = new ArrayList<Position>();
		try {
			//use buffering, reading one line at a time
			//FileReader always assumes default encoding is OK!
//			URL url = this.getClass().getResource(fileName);
//			File file = new File(url.getPath());
				
				// Create FileReader Object and create Buffered Objects
			    BufferedReader inputStream = new BufferedReader(new FileReader(".\\"+fileName));
			    

			try {
				String line = null; //not declared within while loop
		        /*
		        * readLine is a bit quirky :
		        * it returns the content of a line MINUS the newline.
		        * it returns null only for the END of the stream.
		        * it returns an empty String if two newlines appear in a row.
		        */
				String[] temp;				 
				/* delimiter */
				String delimiter = " ";
				
		        while (( line = inputStream.readLine()) != null){
		        	/* given string will be split by the argument delimiter provided. */
					temp = line.split(delimiter);
					positions.add(new Position(Double.parseDouble(temp[6]),Double.parseDouble(temp[7]),Double.parseDouble(temp[8])));
		        }
			}  catch (Exception e) {
					// TODO: handle exception
					System.err.println(e);
			}
		    
			inputStream.close();
		   
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println(e);
		}
		return positions;
	}
	
	public void writeFile(String originalFile,String outputFile,double[][] measurements) {
		try {
			//use buffering, reading one line at a time
			//FileReader always assumes default encoding is OK!
//			URL url1 = this.getClass().getResource(originalFile);
//			File file1 = new File(url1.getPath());
//			URL url2 = this.getClass().getResource(originalFile);
//			File file2 = new File(url2.getPath());
//			BufferedReader input =  new BufferedReader(new FileReader(file1));
//			BufferedWriter out=new BufferedWriter(new FileWriter(file2));
			
			// Create FileReader Object and create Buffered Objects
		    BufferedReader input = new BufferedReader(new FileReader(".\\"+originalFile));;
			BufferedWriter out=new BufferedWriter(new FileWriter(".\\"+outputFile));
			
			try {
				String line = null; //not declared within while loop
		        /*
		        * readLine is a bit quirky :
		        * it returns the content of a line MINUS the newline.
		        * it returns null only for the END of the stream.
		        * it returns an empty String if two newlines appear in a row.
		        */
				int i=0;
	
		        while (((line = input.readLine()) != null) && (i<measurements.length)){
		        	out.write(line+" "+measurements[i][0]+" "+measurements[i][1]+" "+measurements[i][2]+"\r\n");
		        	i++;
		        }
		      }
		      finally {
		        input.close();
		        out.close();
		      }
		} catch (Exception e) {
			// TODO: handle exception
			System.err.println(e);
		}
	}
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		Simulator sim = new Simulator();
		
		ArrayList<Position> a = sim.readFile(args[0]);
		System.out.println("First file read");
		for(int i=0;i<a.size();i++) {
			System.out.println(a.get(i).getTrueX()+" "+a.get(i).getTrueY()+" "+a.get(i).getTrueTheta());
		}
		ArrayList<Position> b = sim.readFile(args[1]);
		System.out.println("Second file read");
		for(int i=0;i<b.size();i++) {
			System.out.println(b.get(i).getTrueX()+" "+b.get(i).getTrueY()+" "+b.get(i).getTrueTheta());
		}
		
		int length = Math.min(a.size(),b.size());
		double[][] measurementsA=new double[length][3];
		double[][] measurementsB=new double[length][3];
		double theta1=0;
		double theta2=0;
		double diffX=0;
		double diffY=0;
		double diffTheta=0;
		double aux1=0;
		double aux2=0;
		Random r = new Random();
		for(int i=0;i<length;i++) {
			theta1=a.get(i).getTrueTheta();
			theta2=b.get(i).getTrueTheta();
			diffX=b.get(i).getTrueX()-a.get(i).getTrueX();
			diffY=b.get(i).getTrueY()-a.get(i).getTrueY();
			diffTheta=theta2-theta1;
			if (diffTheta<-Math.PI) diffTheta+=2*Math.PI;
			measurementsA[i][0]=Math.cos(theta1)*diffX+Math.sin(theta1)*diffY+r.nextGaussian();
			measurementsA[i][1]=-Math.sin(theta1)*diffX+Math.cos(theta1)*diffY+r.nextGaussian();
			aux1=diffTheta+r.nextGaussian();
			if (aux1<-Math.PI) aux1+=2*Math.PI;
			measurementsA[i][2]=aux1;
			measurementsB[i][0]=Math.cos(theta2)*(-diffX)+Math.sin(theta2)*(-diffY)+r.nextGaussian();
			measurementsB[i][1]=-Math.sin(theta2)*(-diffX)+Math.cos(theta2)*(-diffY)+r.nextGaussian();
			aux2=diffTheta+r.nextGaussian();
			if (aux2<-Math.PI) aux2+=2*Math.PI;
			measurementsB[i][2]=aux2;
			System.out.println("A: "+measurementsA[i][0]+" : "+measurementsA[i][1]+" : "+measurementsA[i][2]);
			System.out.println("B: "+measurementsB[i][0]+" : "+measurementsB[i][1]+" : "+measurementsB[i][2]);
		}
		sim.writeFile(args[0],args[2],measurementsA);
		sim.writeFile(args[1],args[3],measurementsB);
	}
}
