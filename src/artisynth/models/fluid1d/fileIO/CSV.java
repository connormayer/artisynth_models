package artisynth.models.fluid1d.fileIO;

import java.io.*;

public class CSV
{
    public double[][] data;
    public String[] headers;
    public boolean hasHeaders = true;		// is the first row a header row?
    public char delim = ',';			// the delimiting character
    public int nCols;				// I is the number of columns
    public int nRows;				// J is the number of rows

    public CSV()
    {
    }
    public CSV(String filename)
    {
        Read(filename);
    }

    public void Read(String filename)
    {
        try
        {
            BufferedReader input = new BufferedReader(new FileReader(filename));
            String inLine = input.readLine();
            String[] inSplit = inLine.split(",");
            nCols = inSplit.length;
            nRows=1;							// one line has already been read
            while (input.ready()==true)
            {
                inLine = input.readLine();
                nRows++;
            }

            //input.reset();
            input.close(); 
            input = new BufferedReader(new FileReader(filename));		// I don't like this!!

            if (hasHeaders == true)
            {
                nRows = nRows-1;
                inLine = input.readLine();
                headers = inLine.split(",");
            }

            //now I know my dimensions
            data = new double[nRows][nCols];

            for (int j=0; j<nRows; j++)
            {
                inLine = input.readLine();
                inSplit = inLine.split(",");
                for (int i=0; i<nCols; i++)
                {
                    data[j][i] = Double.valueOf(inSplit[i]);
                }
            }
        }
        catch(Exception e)
        {
        }

    }

    public static double[][] ReadData(String filename)
    {
        int I;					// I is the number of columns
        int J=1;					// J is the number of rows

        try
        {
            BufferedReader input = new BufferedReader(new FileReader(filename));
            String inLine = input.readLine();
            String[] inSplit = inLine.split(",");
            I = inSplit.length;
            while (input.ready()==true)
            {
                inLine = input.readLine();
                J++;
            }

            //now I know my dimensions
            double[][] data = new double[J][I];

            //input.reset();
            input.close(); 
            input = new BufferedReader(new FileReader(filename));		// I don't like this!!

            for (int j=0; j<J; j++)
            {

                inLine = input.readLine();
                inSplit = inLine.split(",");

                for (int i=0; i<I; i++)
                {
                    data[j][i] = Double.valueOf(inSplit[i]);
                }
            }

            return data;
        }
        catch(Exception e)
        {
            return null;
        }
    }

    public static void Write (String filename, String headers[], double data[][])
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));

            int nCols = data[0].length;
            int nRows = data.length;

            //headers
            if (headers != null)
            {
                for (int a=0; a<nCols; a++)
                {
                    if (a<nCols-1)
                        file.print(headers[a] + ",");
                    else
                        file.print(headers[a] + "\n");
                }
                //file.print("\n");
            }

            //data
            for(int b=0; b<nRows; b++)
            {

                for (int a=0; a<nCols; a++)
                {
                    //file.print(data[b][a] + ",");

                    if (a<nCols-1)
                        file.print(data[b][a] + ",");
                    else
                        file.print(data[b][a] + "\n");

                }
                //file.print("\n");
            }
            file.close();
        }
        catch(Exception e)
        {
        }
    }

    public static void Write (String filename, double data[][])
    {
        String[] headers = null;
        Write(filename, headers, data);
    }

    //-------------------------------------------------------------------------------
    public static void WriteVector (String filename, String header, double[] data)
    {
        //writes a vector out as a single column
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));

            int nRows = data.length;

            //headers
            if (header != null)
            {
                file.print(header + "\n");
            }

            //data
            for(int b=0; b<nRows; b++)
            {
                file.print(data[b] + "\n");
            }
            file.close();
        }
        catch(Exception e)
        {
        }
    }
    public static void WriteVector (String filename, double[] data)
    {
        String header = null;
        WriteVector(filename, header, data);
    }
    //-------------------------------------------------------------------------------
}
