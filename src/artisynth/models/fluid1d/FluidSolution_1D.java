package artisynth.models.fluid1d;

import java.io.*;
import java.util.ArrayList;

import artisynth.models.fluid1d.fluidUtils.FluidUtils;

import maspack.matrix.Point3d;

public class FluidSolution_1D implements FluidSolution
{
    Geometry_1D geometry;
    ArrayList<Field> fields = new ArrayList<Field>();
    ArrayList<Value> values = new ArrayList<Value>();
    String name = "FluidSolution1D";


    public void initialize()
    {
        //geometry.getNumberOfPoints()
        // initialize the default fields? u,p, rho,mu?
    }

    public void createField(String name, int size)
    {
        setField(name, new double[size]);
    }

    public void setField(String name, double[] values)
    {
        // if the field doesn't already exist, it will be created
        int loc = findField(name);
        if (loc == -1)
        {
            Field f = new Field(name, values);
            fields.add(f);
        }
        else
        {
            fields.get(loc).values = values;
        }
    }

    public void setField(String name, int i, double value)
    {
        int loc = findField(name);
        if (loc == -1)
        {
            Field f = new Field(name, getNumberOfPoints());
            f.values[i] = value;
            fields.add(f);
            //System.out.printf("Fluid Solution Error: the field '%s' does not exist\n", name);
        }
        else
        {
            fields.get(loc).values[i] = value;
        }
    }

    public double[] getField(String name)
    {
        int loc = findField(name);
        if (loc == -1)
            //return new double[geometry.getNumberOfPoints()];
            return null;
        else
            return fields.get(loc).values;
    }

    public double getField(String name, int i)
    {
        int loc = findField(name);
        if (loc == -1)
            return Double.NaN;
        else
            return fields.get(loc).values[i];
    }

    //
    public double getFieldInterpolation(String name, Point3d p)
    {
        Centerline cl = geometry.getCenterline();
        double s = cl.findClosestS(p);

        int loc = findField(name);
        if (loc == -1)
            return Double.NaN;
        else
            return FluidUtils.Interpolate1D_linear(cl.getS(), fields.get(loc).values, s);
    }

    public int getNumberOfFields()
    {
        return fields.size();
    }

    public String getFieldName(int i)
    {
        return fields.get(i).name;
    }

    public double[] getFieldData(int i)
    {
        return fields.get(i).values;
    }

    int findField(String name)
    {
        int a=0;
        while (a<fields.size())
        {
            if ( fields.get(a).name.equals(name) )
                return a;
            a++;
        }
        return -1;
    }

    public boolean fieldExists(String name)
    {
        int fi = findField(name);
        if (fi == -1)
            return false;
        else
            return true;
    }

    // -----

    public void setValue(String name, double value)
    {
        // if the field doesn't already exist, it will be created
        int loc = findValue(name);
        if (loc == -1)
        {
            Value v = new Value(name, value);
            values.add(v);
        }
        else
        {
            values.get(loc).value = value;
        }
    }

    public double getValue(String name)
    {
        int loc = findValue(name);
        if (loc == -1)
            return Double.NaN;			// TODO: I don't like this
        else
            return values.get(loc).value;
    }

    int findValue(String name)
    {
        int a=0;
        while (a<values.size())
        {
            if ( values.get(a).name.equals(name) )
                return a;
            a++;
        }
        return -1;
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    // -----

    public void setGeometry(Geometry_1D geom)
    {
        geometry = geom;
    }

    public Geometry_1D getGeometry()
    {
        return geometry;
    }

    public int getNumberOfPoints()
    {
        return geometry.getNumberOfPoints();
    }

    // --- Set up some default fields (I should probably move away from this) --- //

    public double[] getVelocity()
    {
        return getField("u");
    }

    public double getVelocity(int i)
    {
        return getField("u", i);
    }

    public void setVelocity(double[] u)
    {
        setField("u", u);
    }

    public void setVelocity(int i, double u)
    {
        setField("u", i, u);
    }

    public double[] getPressure()
    {
        return getField("p");
    }

    public double getPressure(int i)
    {
        return getField("p", i);
    }

    public void setPressure(double[] p)
    {
        setField("p", p);
    }

    public void setPressure(int i, double p)
    {
        setField("p", i, p);
    }

    public double getDensity()
    {
        return getValue("rho");
    }

    public void setDensity(double rho)
    {
        setValue("rho", rho);
    }

    public double getViscosity()
    {
        return getValue("mu");
    }

    public void setViscosity(double mu)
    {
        setValue("mu", mu);
    }

    // --- --- //
    public void writeSolution(String dir, String filename)
    {
        writeSolutionVTK(dir, filename);		// default??
    }

    public void writeSolutionTXT(String dir, String filename)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(dir + filename + ".txt", false)));

            file.print("Constants\n");
            for (Value v : values)
            {
                file.printf("%s = %f\n", v.name, v.value);
            }
            file.println();

            file.print("Geometry\n");
            Field centerline = new Field("centerline", geometry.getCenterline().distances);
            Field area = new Field("area", geometry.area);
            Field perim = new Field("perim", geometry.perim);
            file.println(centerline);
            file.println(area);
            file.println(perim);
            file.println();

            file.print("Fields\n");
            for (Field f: fields)
            {
                file.println(f.toString());
            }

            file.close();
        }
        catch(Exception e)
        {
        }
    }

    public void writeSolutionCSV(String dir, String filename)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(dir + filename + ".csv", false)));

            int nRows = geometry.getNumberOfPoints();

            // write headers
            file.print("pX");
            file.print(",pY");
            file.print(",pZ");
            file.print(",length");
            file.print(",area");
            file.print(",perim");

            for (Field f: fields)
            {
                file.print("," + f.name );
            }
            file.print("\n");

            // write data
            for (int a=0; a<nRows; a++)
            {
                file.printf( "%f", geometry.getCenterline().getVertices().get(a).x);
                file.printf(",%f", geometry.getCenterline().getVertices().get(a).y);
                file.printf(",%f", geometry.getCenterline().getVertices().get(a).z);
                file.printf(",%f", geometry.getCenterline().getS(a));
                file.printf(",%f", geometry.getArea(a));
                file.printf(",%f", geometry.getPerimeter(a));

                for (Field f: fields)
                {
                    file.printf(",%f", f.values[a] );
                }
                file.print("\n");
            }

            file.close();
        }
        catch(Exception e)
        {
        }
    }

    public void writeSolutionVTK(String dir, String filename)
    {
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(dir + filename + ".vtk", false)));

            // write headers
            file.println("# vtk DataFile Version 3.0");	// header
            file.println("FluidSolution1D");		// title
            file.println("ASCII");				// data type: ASCII or BINARY
            file.println("DATASET POLYDATA");		// dataset type: STRUCTURED_POINTS, STRUCTURED_GRID, UNSTRUCTURED_GRID, POLYDATA, RECTILINEAR_GRID,  FIELD

            int nPoints = geometry.getNumberOfPoints();
            int nLines = nPoints - 1;

            // write points --> //POINTS 19 float
            file.println();
            file.printf("POINTS %d float\n", nPoints);
            for (int a=0; a<nPoints; a++)
            {
                Point3d p = geometry.centerline.getVertices().get(a);
                file.printf("%f %f %f\n", p.x, p.y, p.z);
            }

            // write points --> //POINTS 19 float
            file.println();
            file.printf("LINES %d %d\n", nLines, nLines*3);
            for (int a=0; a<nLines; a++)
            {
                file.printf("2 %d %d\n", a, a+1);
            }

            // write the point data
            int nFields = fields.size() + 3; // fields + length, area and perimeter
            file.println();
            file.printf("POINT_DATA %d\n", nPoints);
            file.printf("FIELD fieldData %d\n", nFields);

            file.printf("%s %d %d float\n", "length", 1, nPoints);
            for (int a=0; a<nPoints; a++)
            {
                file.printf("%f\n", geometry.getCenterline().getS()[a]);
            }

            file.printf("%s %d %d float\n", "area", 1, nPoints);
            for (int a=0; a<nPoints; a++)
            {
                file.printf("%f\n", geometry.getArea()[a]);
            }

            file.printf("%s %d %d float\n", "perim", 1, nPoints);
            for (int a=0; a<nPoints; a++)
            {
                file.printf("%f\n", geometry.getPerimeter()[a]);
            }

            for (Field f : fields)
            {
                file.printf("%s %d %d float\n", f.name, 1, nPoints);
                for (int a=0; a<nPoints; a++)
                {
                    file.printf("%f\n", f.values[a]);
                }
            }

            file.close();
        }
        catch(Exception e)
        {
        }
    }

    public FluidSolution_1D deepCopy()
    {
        FluidSolution_1D fs = new FluidSolution_1D();

        if (geometry != null)
            fs.setGeometry(this.geometry.deepCopy());

        for (Field f : fields)
        {
            fs.setField(f.name, artisynth.models.fluid1d.fluidUtils.FluidUtils.deepCopy_doubleArray(f.values));
        }
        for (Value v : values)
        {
            fs.setValue(v.name, v.value);
        }

        return fs;
    }

    public FluidSolution_1D getSolutionSection(int iStart, int iEnd)
    {
        FluidSolution_1D fs = new FluidSolution_1D();
        fs.setGeometry(this.geometry.getGeometrySection(iStart, iEnd));
        fs.setName(this.name);
        for (Field f : fields)
            fs.fields.add(f.getFieldSection(iStart,iEnd));
        for (Value v : values)
            fs.setValue(v.name, v.value);

        return fs;
    }

    public void setSolutionSection(int iStart, FluidSolution_1D fs)
    {
        // this behavior is a bit uncertain: overwrite geom section? values? For simplicity, I won't.
        for (int a=0; a<fs.getNumberOfFields(); a++)
        {
            double[] arr = this.getField(fs.getFieldName(a));
            double[] arrSec = fs.getFieldData(a);
            for (int b=0; b<fs.getNumberOfPoints(); b++)
            {
                arr[b+iStart] = arrSec[b];
            }
        }
    }

}

class Field
{
    String name;
    double[] values;

    Field(String name, int nValues)
    {
        this.name = name;
        values = new double[nValues];
    }

    Field(String name, double[] values)
    {
        this.name = name;
        this.values = values;
    }

    public String toString()
    {
        String output = String.format("%s:", name);
        for (int a=0; a<values.length; a++)
            output = output + String.format(" %f", values[a]);

        return output;
    }

    public Field getFieldSection(int iStart, int iEnd)
    {
        int N = iEnd-iStart+1;
        double[] arr = new double[N];
        for (int i=0; i<N; i++)
        {
            arr[i] = values[i+iStart];
        }
        return new Field(name, arr);
    }
}

class Value
{
    String name;
    double value;

    Value(String name)
    {
        this.name = name;
    }

    Value(String name, double value)
    {
        this.name = name;
        this.value = value;
    }
}
