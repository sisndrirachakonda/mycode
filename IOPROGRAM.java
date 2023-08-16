import java.io.*;
public class IOPROGRAM
{

	public static void main(String[] args) throws Exception 
	{
	InputStreamReader k=new InputStreamReader(System.in);
	
		BufferedReader r= new BufferedReader(k);
		/*System.out.println("enter some frist num");
	    int s=Integer.parseInt(r.readLine());
	    System.out.println("enter some sec num");
	    int e=Integer.parseInt(r.readLine());
	    int result = s+e;*/
	    System.out.println("enter some frist char");
	    char s=(char)r.read();
	    
	    System.out.println("enter some sec char");
	    char e=(char)r.read();
	    int result =Integer.parseInt() e+s;
	    
		System.out.println("you got "+"String");

	}

}
