package artisynth.models.fluid1d.fileIO;

import java.util.ArrayList;

import artisynth.core.femmodels.FemModel3d;

import maspack.geometry.PolygonalMesh;

public class VTK_IO_Tests
{
    String testDir = "/home/peter/temp/ioTests/";

    public void surfaceMeshTest()
    {
        PolygonalMesh tube = maspack.geometry.MeshFactory.createQuadTube(0.8, 1.0, 4.0, 20, 2, 5);
        int nPoints = tube.numVertices();
        int nFaces = tube.numFaces();

        double[] xField = new double[nPoints];
        double[] yField = new double[nPoints];
        double[] zField = new double[nPoints];
        ArrayList<double[]> locVec = new ArrayList<double[]>();
        ArrayList<double[]> funkyVec = new ArrayList<double[]>();

        for (int a=0; a<nPoints; a++)
        {
            double[] p = {tube.getVertex(a).getPosition().x, tube.getVertex(a).getPosition().y, tube.getVertex(a).getPosition().z};
            xField[a] = p[0];
            yField[a] = p[1];
            zField[a] = p[2];
            locVec.add(p);
            funkyVec.add(new double[]{p[0]*-0.2, p[1]*5.5, 0.1*p[2]});
        }



        //PolygonalMesh tube  = maspack.geometry.MeshFactory.createTube(0.8, 1.0, 4.0, 20, 15, 15);
        VTK_IO vtkIO = new VTK_IO();
        vtkIO.addPointDataScalars("xField", xField);
        vtkIO.addPointDataScalars("yField", yField);
        vtkIO.addPointDataScalars("zField", zField);
        vtkIO.addPointDataVectors("locVector", locVec);
        vtkIO.addPointDataVectors("funkyVector", funkyVec);
        vtkIO.write(testDir + "surf_tube_quad.vtk", tube);

        System.out.println("length of funky vec = " + funkyVec.size());

        //VTK_IO.writeVTK(testDir + "surf_tube_quad.vtk", tube);
        //VTK_IO.writeVTK(testDir + "surf_tube_tri.vtk",  tube);
    }

    public void volumeMeshTest()
    {
        FemModel3d fem = new FemModel3d();
        artisynth.core.femmodels.FemFactory.createHexTube(fem, 4.0, 0.7, 1.1, 10, 9, 9);
        int nPoints = fem.numNodes();
        int nCells  = fem.numElements();

        double[] xField = new double[nPoints];
        double[] yField = new double[nPoints];
        double[] zField = new double[nPoints];
        ArrayList<double[]> locVec = new ArrayList<double[]>();
        ArrayList<double[]> funkyVec = new ArrayList<double[]>();

        for (int a=0; a<nPoints; a++)
        {
            double[] p = {fem.getNode(a).getPosition().x, fem.getNode(a).getPosition().y, fem.getNode(a).getPosition().z};
            xField[a] = p[0];
            yField[a] = p[1];
            zField[a] = p[2];
            locVec.add(p);
            funkyVec.add(new double[]{p[0]*-0.2, p[1]*5.5, 0.1*p[2]});
        }

        //PolygonalMesh tube  = maspack.geometry.MeshFactory.createTube(0.8, 1.0, 4.0, 20, 15, 15);
        VTK_IO vtkIO = new VTK_IO();
        vtkIO.addPointDataScalars("xField", xField);
        vtkIO.addPointDataScalars("yField", yField);
        vtkIO.addPointDataScalars("zField", zField);
        vtkIO.addPointDataVectors("locVector", locVec);
        vtkIO.addPointDataVectors("funkyVector", funkyVec);
        vtkIO.write(testDir + "vol_tube_quad.vtk", fem);

        //VTK_IO.writeVTK(testDir + "surf_tube_quad.vtk", tube);
        //VTK_IO.writeVTK(testDir + "surf_tube_tri.vtk",  tube);
    }
}
