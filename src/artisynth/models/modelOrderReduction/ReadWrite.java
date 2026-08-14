package artisynth.models.modelOrderReduction;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.LineNumberReader;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;

import artisynth.core.femmodels.AnsysWriter;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dList;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.femmodels.TetElement;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.RenderableComponentBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.frank2.GenericModel;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.DenseMatrixBase;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public class ReadWrite {

   public ReadWrite () {
      // TODO Auto-generated constructor stub
   }
   
   public static ArrayList<String []> readScript(String filename)
   {   
      ArrayList<String []> DataList = new ArrayList<String []> ();

      try 
      {
         BufferedReader file = new BufferedReader(new FileReader(filename));
         String inLine = null;
         String[] inSplit = null;

         while (file.ready() == true)
         {
            inLine = file.readLine();
            inLine = inLine.trim();
            if (inLine.isEmpty() == true)
            {
               continue; // ignore
            }
            else if (inLine.startsWith("#") == true) {
               continue;   // comments are ignored
            }
            else 
            {
               inSplit = inLine.split(",");
               for (String str :  inSplit) {
                  str.trim ();
               }
               DataList.add (inSplit);
            }
         }
         file.close();
      }
      catch(Exception e) 
      {
         e.printStackTrace();
      }
      return DataList;
   }
   
   public static void readMatrix(DenseMatrixBase rMat, String filePath) throws IOException {

      FileReader fr = null;
      LineNumberReader lnr = null;
      String str;
      String [] strs;

      ArrayList<double []> output = new ArrayList<double []>();
      double tmp;
      int colSize = 1;

      try {
         fr = new FileReader(filePath);
         lnr = new LineNumberReader(fr);

         while((str = lnr.readLine ()) != null) {
            int idx = lnr.getLineNumber ()-1;
            // information in each line
            strs = str.split (" ");
            if (strs.length != 0) {
               if (strs.length > colSize) {
                  colSize = strs.length;
               }
               double [] tmps=  new double [strs.length];
               output.add (tmps);

               for (int i =0; i < strs.length; i++) {
                  // get each element
                  try {
                     tmp = Double.parseDouble (strs[i]);
                     output.get (idx)[i] = tmp;
                  } catch(NumberFormatException e) {
                     output.get (idx)[i] = Double.NaN;
                  }
               }
            }
            else {
               System.out.println("Read " + filePath + 
                  ": There is no content in line " +
                  idx);
               double [] tmps = new double [1];
               tmps[0] = Double.NaN;
               output.add (tmps);
            }
         }

         rMat.setSize (output.size (), colSize);
         for (int i = 0; i< rMat.rowSize (); i++) {
            for (int j = 0; j < rMat.colSize (); j++) {
               if (j <= output.get (i).length-1) {
                  rMat.set (i, j, output.get (i)[j]);
               }
               else {
                  rMat.set (i, j, Double.NaN);
               }
            }
         }

      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      finally {
         // closes the stream and releases system resources
         if(fr!=null)
            fr.close();
         if(lnr!=null)
            lnr.close();
      }
   }

   public static void readMatrix(DenseMatrixBase rMat, Class<?> obj, String name) throws IOException {
      String filePath =
      ArtisynthPath.getSrcRelativePath (
         obj, name);
      readMatrix (rMat, filePath);
   }
   
   public static void readVector(VectorNd rVec, Class<?> obj, String fileName) throws IOException {
      String [] strs = ReadWrite.readStringArray (obj, fileName);
      rVec.setSize (strs.length);
      double tmp;
      
      for (int i =0; i < strs.length; i++) {
         try {
            tmp = Double.parseDouble (strs[i]);
            rVec.set (i, tmp);
         } catch(NumberFormatException e) {
            rVec.set (i, Double.NaN);
         }
      }
   }
   
   public static double [][] readArray(String fileName) throws IOException {
      String filePath = fileName;

      FileReader fr = null;
      LineNumberReader lnr = null;
      String str;
      String [] strs;

      ArrayList<double []> output = new ArrayList<double []>();

      double tmp;
      int colSize = 1;

      try {
         fr = new FileReader(filePath);
         lnr = new LineNumberReader(fr);

         while((str = lnr.readLine ()) != null) {
            int idx = lnr.getLineNumber ()-1;
            // information in each line
            strs = str.split (" ");
            if (strs.length != 0) {
               double [] tmps=  new double [strs.length];
               output.add (tmps);

               for (int i =0; i < strs.length; i++) {
                  // get each element
                  try {
                     tmp = Double.parseDouble (strs[i]);
                     output.get (idx)[i] = tmp;
                  } catch(NumberFormatException e) {
                     output.get (idx)[i] = Double.NaN;
                  }
               }
            }
            else {
               System.out.println("Read " + filePath + 
                  ": There is no content in line " +
                  idx);
               double [] tmps = new double [1];
               tmps[0] = Double.NaN;
               output.add (tmps);
            }
         }

      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      finally {
         // closes the stream and releases system resources
         if(fr!=null)
            fr.close();
         if(lnr!=null)
            lnr.close();
      }

      double [][] results = new double [output.size ()][];
      for (int i = 0; i < output.size (); i ++) {
         results[i] = new double [output.get (i).length];
         for (int j = 0; j < output.get (i).length; j++) {
            results[i][j] = output.get (i)[j];
         }
      }
      return results;
   }
   

   public static double [][] readArray(Class<?> obj, String fileName) throws IOException {
      String filePath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      FileReader fr = null;
      LineNumberReader lnr = null;
      String str;
      String [] strs;

      ArrayList<double []> output = new ArrayList<double []>();

      double tmp;
      int colSize = 1;

      try {
         fr = new FileReader(filePath);
         lnr = new LineNumberReader(fr);

         while((str = lnr.readLine ()) != null) {
            int idx = lnr.getLineNumber ()-1;
            // information in each line
            strs = str.split (" ");
            if (strs.length != 0) {
               double [] tmps=  new double [strs.length];
               output.add (tmps);

               for (int i =0; i < strs.length; i++) {
                  // get each element
                  try {
                     tmp = Double.parseDouble (strs[i]);
                     output.get (idx)[i] = tmp;
                  } catch(NumberFormatException e) {
                     output.get (idx)[i] = Double.NaN;
                  }
               }
            }
            else {
               System.out.println("Read " + filePath + 
                  ": There is no content in line " +
                  idx);
               double [] tmps = new double [1];
               tmps[0] = Double.NaN;
               output.add (tmps);
            }
         }

      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      finally {
         // closes the stream and releases system resources
         if(fr!=null)
            fr.close();
         if(lnr!=null)
            lnr.close();
      }

      double [][] results = new double [output.size ()][];
      for (int i = 0; i < output.size (); i ++) {
         results[i] = new double [output.get (i).length];
         for (int j = 0; j < output.get (i).length; j++) {
            results[i][j] = output.get (i)[j];
         }
      }
      return results;
   }

   public static String [] readStringArray(Class<?> obj, String fileName) throws IOException {
      String filePath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      FileReader fr = null;
      LineNumberReader lnr = null;
      String str;
      ArrayList<String> output = new ArrayList<String>();
      String [] strs;

      try {
         fr = new FileReader(filePath);
         lnr = new LineNumberReader(fr);

         while((str = lnr.readLine ()) != null) {
            // information in each line
            output.add (str);
         }
      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      finally {
         // closes the stream and releases system resources
         if(fr!=null)
            fr.close();
         if(lnr!=null)
            lnr.close();
      }
      strs = new String [output.size ()];
      for (int i = 0; i < strs.length; i++) {
         strs[i] = output.get (i);
      }
      return strs;
   }
   
   public static String [] readStringArray(String fileName) throws IOException {
      String filePath = fileName;

      FileReader fr = null;
      LineNumberReader lnr = null;
      String str;
      ArrayList<String> output = new ArrayList<String>();
      String [] strs;

      try {
         fr = new FileReader(filePath);
         lnr = new LineNumberReader(fr);

         while((str = lnr.readLine ()) != null) {
            // information in each line
            output.add (str);
         }
      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      finally {
         // closes the stream and releases system resources
         if(fr!=null)
            fr.close();
         if(lnr!=null)
            lnr.close();
      }
      strs = new String [output.size ()];
      for (int i = 0; i < strs.length; i++) {
         strs[i] = output.get (i);
      }
      return strs;
   }
   
   public static String [][] readStringDoubleArray(Class<?> obj, String fileName) throws IOException {
      String filePath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      FileReader fr = null;
      LineNumberReader lnr = null;
      String str;
      String [] strArr;
      ArrayList<String []> output = new ArrayList<String []>();
      String [][] strs;

      try {
         fr = new FileReader(filePath);
         lnr = new LineNumberReader(fr);

         while((str = lnr.readLine ()) != null) {
            strArr = str.split (" ");
            // information in each line
            output.add (strArr);
         }
      }
      catch (FileNotFoundException e) {
         e.printStackTrace();
      }
      finally {
         // closes the stream and releases system resources
         if(fr!=null)
            fr.close();
         if(lnr!=null)
            lnr.close();
      }
      strs = new String [output.size ()][];
      for (int i = 0; i < strs.length; i++) {
         strs[i] = output.get (i);
      }
      return strs;
   }

   public static void writeMatrixToFile(DenseMatrixBase argM, Class<?> obj, String fileName) throws IOException {
      double [][] temMArray = new double [argM.rowSize ()][argM.colSize ()];
      argM.get (temMArray);
      //System.out.println("\nWriting Nd matrix to file...");

      writeArrayToFile (temMArray, obj, fileName);
   }
   
   public static void writeMatrixToFile(DenseMatrixBase argM, String fileName) throws IOException {
      double [][] temMArray = new double [argM.rowSize ()][argM.colSize ()];
      argM.get (temMArray);
      //System.out.println("\nWriting Nd matrix to file...");

      writeArrayToFile (temMArray, fileName);
   }
   
   public static void writeArrayToFile(double [][] argArray, String fileName) throws IOException {

      PrintWriter writer = new PrintWriter(fileName, "UTF-8");

      for (int i = 0; i < argArray.length; i++) {
         for (int j = 0; j < argArray[i].length; j++) {
            writer.print (argArray[i][j] + " ");
            //System.out.println (argArray[i][j]);
         }
         writer.print ("\n");
         //System.out.print("\n");
      }
      //System.out.println ("Array has been written to file.");
      writer.close ();

   }

   public static void writeArrayToFile(double [][] argArray, Class<?> obj, String fileName) throws IOException {
      //System.out.println("Writing array to file: ");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");

      for (int i = 0; i < argArray.length; i++) {
         for (int j = 0; j < argArray[i].length; j++) {
            writer.print (argArray[i][j] + " ");
            //System.out.println (argArray[i][j]);
         }
         writer.print ("\n");
         //System.out.print("\n");
      }
      //System.out.println ("Array has been written to file.");
      writer.close ();

   }

   public static void writeArrayToFile(double [] argArray, Class<?> obj, String fileName) throws IOException {

      System.out.println("Writing double array to file:");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      for (double temData : argArray) {
         writer.println ( temData );
         System.out.println (temData);
      }
      writer.close ();
      System.out.println ("\nArray has been written to file!");
   }
   
   public static void writeArrayToFile(double [] argArray, String fileName) throws IOException {

      System.out.println("Writing double array to file:");

      // load the mesh
      String meshPath = fileName;
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      for (double temData : argArray) {
         writer.println ( temData );
         System.out.println (temData);
      }
      writer.close ();
      System.out.println ("\nArray has been written to file!");
   }

   public static void writeArrayToFile(String [] argArray, Class<?> obj, String fileName) throws IOException {

      System.out.println("Writing String array to file:");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      for (String temData : argArray) {
         writer.println ( temData );
         System.out.println (temData+" ");
      }
      writer.close ();
      System.out.println ("\nString array has been written to file!");
   }
   
   public static void writeArrayToFile(String [] argArray, String fileName) throws IOException {

      System.out.println("Writing String array to file:");

      // load the mesh
      String meshPath = fileName;
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      for (String temData : argArray) {
         writer.println ( temData );
         System.out.println (temData+" ");
      }
      writer.close ();
      System.out.println ("\nString array has been written to file!");
   }
   
   public static void writeArrayToFile(String [][] argArray, Class<?> obj, String fileName) throws IOException {

      System.out.println("Writing String array to file:");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      for (String [] strArr : argArray) {
         String strs = new String();
         for (String str : strArr) {
             strs = strs + str + " ";
         }
         writer.println ( strs );
         //System.out.println (strs);
      }
      writer.close ();
      System.out.println ("\nString array has been written to file!");
   }
   
   public static void writeArrayToFile(String [][] argArray, String fileName) throws IOException {

      System.out.println("Writing String array to file:");

      // load the mesh
      String meshPath = fileName;
      File temf = new File(meshPath);
      temf.createNewFile ();

      PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      for (String [] strArr : argArray) {
         String strs = new String();
         for (String str : strArr) {
             strs = strs + str + " ";
         }
         writer.println ( strs );
         //System.out.println (strs);
      }
      writer.close ();
      System.out.println ("\nString array has been written to file!");
   }

   //--------------------------------------------------
   // write FEM
   //--------------------------------------------------

   public static void writeNodesToAnsys(FemModel3d fem, Class<?> obj, String fileName) throws IOException {
      System.out.println("Writing nodes to ANSYS file: \n");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      //PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      AnsysWriter.writeNodeFile (fem, meshPath);
   }
   
   public static void writeNodesToAnsys(FemModel3d fem, String fileName) throws IOException {
      System.out.println("Writing nodes to ANSYS file: \n");

      //PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      AnsysWriter.writeNodeFile (fem, fileName);
   }

   public static void writeNodesToMdl(FemModel3d fem, Class<?> obj, String fileName) throws IOException {
      System.out.println("Writing nodes to ANSYS file: \n");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      PrintWriter pw = new PrintWriter(meshPath, "UTF-8");
      pw.println ("[Name,STRING]");
      pw.println ("../Data/merged.hdr_CP_AFFINE.mdl\n");
      pw.println ("[Vertices, ARRAY1<POINT3D>]");
      pw.println (Integer.toString (fem.getNodes ().size ()));
      for (FemNode3d n : fem.getNodes()) {
         pw.println(n.getRestPosition ().toString("%f "));
      }
      pw.flush();
      pw.close ();
      //writer = new PrintWriter(meshPath+"_non2", "UTF-8");
      //fem.printNodes1 (writer);
   }

   public static void writeElementToAnsys(FemModel3d fem, String fileName) throws IOException {
      System.out.println("Writing elements to ANSYS file: \n");

      //PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      AnsysWriter.writeElemFile (fem, fileName);
   }
   
   public static void writeElementToAnsys(FemModel3d fem, Class<?> obj, String fileName) throws IOException {
      System.out.println("Writing elements to ANSYS file: \n");

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      //PrintWriter writer = new PrintWriter(meshPath, "UTF-8");
      AnsysWriter.writeElemFile (fem, meshPath);
   }

   public static void writeElementsToM3d(FemModel3d fem, Class<?> obj, String fileName) throws FileNotFoundException, UnsupportedEncodingException {
      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, fileName);

      PrintWriter pw = new PrintWriter(meshPath, "UTF-8");
      int femIdx = 1;
      // get element list
      FemElement3dList<FemElement3d>myElements = fem.getElements();
         //(FemElement3dList<FemElement3d>)fem.getElements();
      PointList<FemNode3d> myNodes = fem.getNodes ();

      pw.println ("[Elements, ARRAY1<STRING>]");
      pw.println (Integer.toString (fem.getElements ().size ()));
      for (FemElement3d e : myElements) {
         FemNode3d[] nodes = e.getNodes();
         int[] nums = new int[16];

         if (e instanceof HexElement) {
            nums[0] = myNodes.indexOf(nodes[0]) + 1;
            nums[1] = myNodes.indexOf(nodes[1]) + 1;
            nums[2] = myNodes.indexOf(nodes[2]) + 1;
            nums[3] = myNodes.indexOf(nodes[3]) + 1;
            nums[4] = myNodes.indexOf(nodes[4]) + 1;
            nums[5] = myNodes.indexOf(nodes[5]) + 1;
            nums[6] = myNodes.indexOf(nodes[6]) + 1;
            nums[7] = myNodes.indexOf(nodes[7]) + 1;
            nums[8] = myNodes.indexOf(nodes[0]) + 1;
            nums[9] = myNodes.indexOf(nodes[1]) + 1;
            nums[10] = myNodes.indexOf(nodes[2]) + 1;
            nums[11] = myNodes.indexOf(nodes[3]) + 1;
            nums[12] = myNodes.indexOf(nodes[4]) + 1;
            nums[13] = myNodes.indexOf(nodes[5]) + 1;
            nums[14] = myNodes.indexOf(nodes[6]) + 1;
            nums[15] = myNodes.indexOf(nodes[7]) + 1;
         }
         else {
            throw new IllegalArgumentException("Unknown element type: "
            + e.getClass().getName());
         }
         for (int i = 0; i < nums.length; i++) {
            pw.print(nums[i] + " ");
         }
         pw.println();
         femIdx++;
      }
      pw.flush();
      pw.close ();
   }

   //--------------------------------------------------
   // read and write Mesh
   //--------------------------------------------------

   public static void addMesh (RenderableComponentList<RenderableComponentBase> MeshCptList, Class<?> objPath, String argFileNameWithoutIndex, int quantity /*The number of mesh*/, int startIdx /*The start index*/, double argScaling) throws IOException {
      String fileNameSuffix = null;

      if (argFileNameWithoutIndex.endsWith(".obj") == true) {
         argFileNameWithoutIndex = argFileNameWithoutIndex.replace (".obj", "");
         fileNameSuffix = ".obj";
      }
      else if (argFileNameWithoutIndex.endsWith(".ply") == true) {
         argFileNameWithoutIndex = argFileNameWithoutIndex.replace (".ply", "");
         fileNameSuffix = ".ply";
      }
      else if (argFileNameWithoutIndex.endsWith(".stl") == true) {
         argFileNameWithoutIndex = argFileNameWithoutIndex.replace (".stl", "");
         fileNameSuffix = ".stl";
      }
      else if (argFileNameWithoutIndex.endsWith (".vtk") == true) {
         argFileNameWithoutIndex = argFileNameWithoutIndex.replace (".vtk", "");
         fileNameSuffix = ".vtk";
      }

      String tmpName = argFileNameWithoutIndex.split ("_", 2)[0];
      for (int i = startIdx; i <= quantity-1+startIdx; i++) {
         PolygonalMesh tmpMesh = readMesh(objPath,
            argFileNameWithoutIndex+'_'+Integer.toString (i+1) + fileNameSuffix, 
            argScaling);
         RigidBody MeshCpt = new RigidBody(tmpName + Integer.toString (i));
         MeshCpt.setMesh (tmpMesh);
         MeshCptList.add (MeshCpt);
      }
   }

   public static void addMesh(RenderableComponentList<RenderableComponentBase> MeshCptList, Class<?> objPath, String argFileNameWithoutIndex, int quantity /*The number of mesh*/, double argScaling) throws IOException {
      addMesh(MeshCptList, objPath, argFileNameWithoutIndex, quantity, 0, argScaling);
   }

   public static void addMesh(RenderableComponentList<RenderableComponentBase> MeshCptList, Class<?> objPath, String argFileNameWithoutIndex, int quantity /*The number of mesh*/) throws IOException {
      addMesh(MeshCptList, objPath, argFileNameWithoutIndex, quantity, 1.0);
   }

   public static void addMesh(RenderableComponentList<RenderableComponentBase> MeshCptList, Class<?> objPath, String argFileNameWithoutIndex) throws IOException {
      addMesh(MeshCptList, objPath, argFileNameWithoutIndex, 1);
   }

   /**
    * 
    * @param name
    * @param fileName
    * @return skinMeshBody
    * @throws IOException
    */
   public static PolygonalMesh readMesh (Class<?> obj, String fileName, double argScaling) throws IOException {

      // load the mesh
      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, "/");
      PolygonalMesh temMesh = GenericModel.loadGeometry (meshPath, fileName);
      //change mesh scale
      temMesh.scale (argScaling);

      return temMesh;
   }

   public static PolygonalMesh readMesh (Class<?> obj, String fileName) throws IOException {
      return readMesh(obj, fileName, 1.0);
   }
   
   public static void writeMesh(PolygonalMesh argMesh, Class<?> obj, String fileName) throws IOException {

      String meshPath =
      ArtisynthPath.getSrcRelativePath (
         obj, "/");

      File temF = new File(meshPath + fileName);
      PolygonalMesh mesh = argMesh.copy ();
      
      argMesh.setFixed (false);
      if (!mesh.meshToWorldIsIdentity ()) {
         mesh.transform (mesh.getMeshToWorld ());
      }
      mesh.write (temF);
      //argMesh.writeWorld (ps);
   }
   
   //--------------------------------------------------
   // read and write probe file
   //--------------------------------------------------
   
   public static void writeProbeFile(MatrixNd targetData, double [] times, double startTime, double stopTime, Class<?> obj, String fileName) throws IOException {
      String interpolation = "Cubic";
      int vsize = targetData.colSize ();
      if (targetData.rowSize () == times.length) {
         String [][] probeData = new String [2+times.length][];
         
         // probe head
         probeData[0] = new String [3];
         probeData[1] = new String [3];
         probeData[0][0] = Double.toString (startTime);
         probeData[0][1] = Double.toString (stopTime);
         probeData[0][2] = "1.0";
         probeData[1][0] = interpolation;
         probeData[1][1] = Integer.toString (vsize);
         probeData[1][2] = "explicit";
         
         // probe data
         for (int i = 0; i < times.length; i++) {
            probeData[i+2] = new String [vsize+1];
            probeData[i+2][0] = Double.toString (times[i]);
            for (int j = 0; j < vsize; j++) {
               probeData[i+2][j+1] = Double.toString (targetData.get (i, j));
            }
         }
         ReadWrite.writeArrayToFile (probeData, obj, fileName);
      } else {
         System.out.println ("wirteProbeFile: failed to write the probe file");
      }

   }
   
   public static void writeProbeFile(MatrixNd targetData, double [] times, double startTime, double stopTime, String fileName) throws IOException {
      String interpolation = "Linear";
      int vsize = targetData.colSize ();
      if (targetData.rowSize () == times.length) {
         String [][] probeData = new String [2+times.length][];
         
         // probe head
         probeData[0] = new String [3];
         probeData[1] = new String [3];
         probeData[0][0] = Double.toString (startTime);
         probeData[0][1] = Double.toString (stopTime);
         probeData[0][2] = "1.0";
         probeData[1][0] = interpolation;
         probeData[1][1] = Integer.toString (vsize);
         probeData[1][2] = "explicit";
         
         // probe data
         for (int i = 0; i < times.length; i++) {
            probeData[i+2] = new String [vsize+1];
            probeData[i+2][0] = Double.toString (times[i]);
            for (int j = 0; j < vsize; j++) {
               probeData[i+2][j+1] = Double.toString (targetData.get (i, j));
            }
         }
         ReadWrite.writeArrayToFile (probeData, fileName);
      } else {
         System.out.println ("wirteProbeFile: failed to write the probe file");
      }

   }



}
