package artisynth.models.frank2;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashMap;

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.materials.AxialMaterial;
import artisynth.core.materials.AxialMuscleMaterial;
import artisynth.core.materials.BlemkerAxialMuscle;
import artisynth.core.materials.ConstantAxialMuscle;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.materials.PeckAxialMuscle;
import artisynth.core.mechmodels.DynamicAttachment;
import artisynth.core.mechmodels.DynamicComponent;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointAttachable;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentList;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.models.frank2.frankUtilities.StopWatch.Units;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.NURBSCurve3d;
import maspack.geometry.BVFeatureQuery.InsideQuery;
import maspack.matrix.Point3d;
import maspack.matrix.Vector4d;
import maspack.properties.PropertyInfo;

public class FrankMuscles 
{
    // TODO: options to override the point data read from file (such as body to attach to)
    // TODO: add more flexible symmetry options --> mirror and add to single muscle group, mirror a muscle bundle, etc...
    // TODO: do not write left-side muscles
    
    public enum MuscleSymmetry {none, xSymm, ySymm, zSymm};
    
    
    protected static AxialMaterial blemkerMuscle(double damping, double maxForce, double passiveFraction, 
        double optLengthRatio, double maxLengthRatio, double tendonRatio, double forceScaling)
    {
        BlemkerAxialMuscle mat = new BlemkerAxialMuscle();
        mat.setMaxForce(maxForce);
        mat.setOptLength(optLengthRatio); // TODO: will be scaled...find a better way.
        mat.setMaxLength(maxLengthRatio); // TODO: will be scaled
        mat.setTendonRatio(tendonRatio);
        mat.setDamping(damping);
        mat.setPassiveFraction (passiveFraction);
        mat.setForceScaling(forceScaling);
        return mat;
    }
    
    public static AxialMaterial peckMuscle(double damping, double maxForce, double passiveFraction, 
        double optLengthRatio, double maxLengthRatio, double tendonRatio, double forceScaling)
    {
        PeckAxialMuscle mat = new PeckAxialMuscle();
        mat.setMaxForce(maxForce);
        mat.setOptLength(optLengthRatio); // TODO: will be scaled...find a better way.
        mat.setMaxLength(maxLengthRatio); // TODO: will be scaled
        mat.setTendonRatio(tendonRatio);
        mat.setDamping(damping);
        mat.setPassiveFraction (passiveFraction);
        mat.setForceScaling(forceScaling);
        return mat;
    }

    public static AxialMaterial constantMuscle(double damping, double maxForce, double passiveFraction, 
        double optLengthRatio, double maxLengthRatio, double tendonRatio, double forceScaling)
    {
        ConstantAxialMuscle mat = new ConstantAxialMuscle();
        mat.setMaxForce(maxForce);
        mat.setOptLength(optLengthRatio); // TODO: will be scaled...find a better way.
        mat.setMaxLength(maxLengthRatio); // TODO: will be scaled
        mat.setTendonRatio(tendonRatio);
        mat.setDamping(damping);
        mat.setPassiveFraction (passiveFraction);
        mat.setForceScaling(forceScaling);
        return mat;
    }
    
    public static AxialMaterial linearMaterial(double stiffness, double damping)
    {
        LinearAxialMaterial mat = new LinearAxialMaterial();
        mat.setStiffness(stiffness);
        mat.setDamping(damping);
        return mat;
    }
    
    public static MuscleBundle combineMuscleBundles(MuscleBundle mb1, MuscleBundle mb2)
    {
        MuscleBundle mb = new MuscleBundle();
        for (Muscle m : mb1.getFibres())
        {
            mb.addFibre(m);
        }
        for (Muscle m : mb2.getFibres())
        {
            mb.addFibre(m);
        }
        return mb;
    }

    public static void addMuscleBundles(ComponentList<MuscleBundle> muscles, 
        String name, String filename, AxialMaterial muscleMaterial, MuscleSymmetry muscleSymmetry,
        MechModel mechModel, ComponentList<? extends FemModel3d> fems, ComponentList<? extends RigidBody> rbs)
    {
        MuscleBundleData mbd = readMuscleBundle(filename);
        if (muscleSymmetry == MuscleSymmetry.none)
            ;
        else if (muscleSymmetry == MuscleSymmetry.xSymm)
            mbd.mirrorPoints(0);
        else if (muscleSymmetry == MuscleSymmetry.ySymm)
            mbd.mirrorPoints(1);
        else if (muscleSymmetry == MuscleSymmetry.zSymm)
            mbd.mirrorPoints(2);
        
        muscles.add( loadMuscleBundle(name, mbd, muscleMaterial, mechModel, fems, rbs) );                
    }
    
    public static void addMuscleBundles(FemMuscleModel parentFem, 
        String name, String filename, AxialMaterial muscleMaterial, MuscleSymmetry muscleSymmetry,
        MechModel mechModel, ComponentList<? extends FemModel3d> fems, ComponentList<? extends RigidBody> rbs)
    {
        MuscleBundleData mbd = readMuscleBundle(filename);
        if (muscleSymmetry == MuscleSymmetry.none)
            ;
        else if (muscleSymmetry == MuscleSymmetry.xSymm)
            mbd.mirrorPoints(0);
        else if (muscleSymmetry == MuscleSymmetry.ySymm)
            mbd.mirrorPoints(1);
        else if (muscleSymmetry == MuscleSymmetry.zSymm)
            mbd.mirrorPoints(2);
        
        addMuscleBundle(parentFem, name, mbd, muscleMaterial, mechModel, fems, rbs);                
    }
    
    public static MuscleBundle loadMuscleBundle(String name, String filename, AxialMaterial muscleMaterial,
        MechModel mechModel, ComponentList<? extends FemModel3d> fems, ComponentList<? extends RigidBody> rbs)
    {
        MuscleBundleData mbd = readMuscleBundle(filename);
        return loadMuscleBundle(name, mbd, muscleMaterial, mechModel, fems, rbs);
    }
    
    public static MuscleBundle loadMuscleBundle(String name, MuscleBundleData mbd, AxialMaterial muscleMaterial,
        MechModel mechModel, ComponentList<? extends FemModel3d> fems, ComponentList<? extends RigidBody> rbs)
    {
        MuscleBundle mb = new MuscleBundle();
        mb.setName(name);

        // create the fem markers, or optionally, external points
        int nPoints = mbd.musclePoints.size();
        int nFibres = mbd.connections.size();
        ArrayList<Point> bPoints = new ArrayList<Point>(nPoints);
        
        //BVFeatureQuery query = new BVFeatureQuery();

        for (int a=0; a<nPoints; a++)
        {
            Point3d pnt3d = mbd.musclePoints.get(a).loc;
            Point point = null;
            
            Particle part = new Particle();
            part.setPosition(pnt3d);
            part.setMass(0.0); // the particle itself should not effect the simulation.
            part.setPointDamping(0.0);
            mechModel.addParticle(part);
            point = part;
            
            
            if ( fems.get( mbd.musclePoints.get(a).modelName ) != null )
            {
                FemModel3d fem = fems.get( mbd.musclePoints.get(a).modelName );
                
                mechModel.attachPoint(part, fem);
                // TODO: not sure if I like this check! Slows things down.
//                if (query.isInsideMesh(fem.getSurfaceMesh(), part.getPosition(), 0.5/1000.0) == InsideQuery.INSIDE)
//                    mechModel.attachPoint(part, fem);
//                else
//                    part.setDynamic(false);
            }
            else if (rbs.get( mbd.musclePoints.get(a).modelName ) != null )
            {
                RigidBody rb = rbs.get( mbd.musclePoints.get(a).modelName );
//                point = mechModel.addFrameMarker(rb, pnt3d);
//                mechModel.addParticle(part);
                mechModel.attachPoint(part, rb);
            }
            else
            {
                //point = mechModel.addFrameMarker(rbs.findComponent("ground"), pnt3d);
//                mechModel.addParticle(part);
                part.setDynamic(false);
            }
            point.setName(String.format("%s_pnt%d", name, a));
            bPoints.add(point); 
        }

        for (int a=0; a<nFibres; a++)
        {
            int[] conn = mbd.connections.get(a); 
            for (int b=1; b<conn.length; b++)
            {
                Muscle fibre = new Muscle();
                fibre.setFirstPoint  ( bPoints.get(conn[b-1]) );
                fibre.setSecondPoint ( bPoints.get(conn[b]  ) );
                fibre.setRestLengthFromPoints();

                if (muscleMaterial instanceof AxialMuscleMaterial)
                {
                    AxialMuscleMaterial matCopy = (AxialMuscleMaterial)muscleMaterial.clone();
                    // TODO: opt and max length defined as optLength/length and maxLength/length in "mat"; I don't like this!
                    matCopy.setMaxLength(fibre.getLength() * matCopy.getMaxLength());  
                    matCopy.setOptLength(fibre.getLength() * matCopy.getOptLength());
                    fibre.setMaterial(matCopy);
                }
                else
                {
                    fibre.setMaterial(muscleMaterial.clone());
                }
                mb.addFibre (fibre);
            }
        }

        mb.setFibresActive(true);
        
        return mb;
    }
    
    public static MuscleBundle addMuscleBundle(FemMuscleModel parentModel, String name, MuscleBundleData mbd, AxialMaterial muscleMaterial,
        MechModel mechModel, ComponentList<? extends FemModel3d> fems, ComponentList<? extends RigidBody> rbs)
    {
        MuscleBundle mb = new MuscleBundle();
        mb.setName(name);

        // create the fem markers, or optionally, external points
        int nPoints = mbd.musclePoints.size();
        int nFibres = mbd.connections.size();
        ArrayList<Point> bPoints = new ArrayList<Point>(nPoints);
        
        //BVFeatureQuery query = new BVFeatureQuery();

        // the markers/containing elements are cached and added at once at the end, this saves the BV tree from being continually re-generated
        LinkedHashMap<FemMarker, FemElement3dBase> markerMap = new LinkedHashMap<>();  // large time savings  
        LinkedHashMap<FemNode3d, PointAttachable> nodeMap = new LinkedHashMap<>(); // modest time savings
        
        for (int a=0; a<nPoints; a++)
        {
            Point3d pnt3d = mbd.musclePoints.get(a).loc;
            Point point = null;
            String pName = String.format("%s_pnt%d", name, a);
            
            if ( fems.get( mbd.musclePoints.get(a).modelName ) != null )
            {
                FemModel3d fem = fems.get( mbd.musclePoints.get(a).modelName );
                //if (fem.getName().equals(parentModel.getName()) == true)
                if (fem == parentModel)
                {
                    Point3d pntNew = new Point3d();
                    FemElement3dBase elem = fem.findNearestElement(pntNew, pnt3d);
                    FemMarker marker = new FemMarker(pntNew);
                    markerMap.put(marker, elem);
                    point = marker;
                    // add as marker to parent fem
                    //point = parentModel.addMarker(pnt3d);
                }
                else
                {
                    // attach 
                    FemNode3d node = new FemNode3d(pnt3d);
                    point = node;
                    nodeMap.put(node, fem);
//                    parentModel.addNode(node);
//                    mechModel.attachPoint(point, fem);
                }
            }
            else if (rbs.get( mbd.musclePoints.get(a).modelName ) != null )
            {
                RigidBody rb = rbs.get( mbd.musclePoints.get(a).modelName );
                
                FemNode3d node = new FemNode3d(pnt3d);
                point = node;
                nodeMap.put(node, rb);
//                parentModel.addNode(node);
//                mechModel.attachPoint(point, rb);
            }
            else
            {
                FemNode3d node = new FemNode3d(pnt3d);
                node.setDynamic(false);
                point = node;
                nodeMap.put(node, null);
//                parentModel.addNode(node);
            }
            point.setName(pName);
            bPoints.add(point); 
        }
        
        for (FemMarker marker : markerMap.keySet())
            parentModel.addMarker(marker, markerMap.get(marker));
        for (FemNode3d node : nodeMap.keySet())
        {
            parentModel.addNode(node);
            if (nodeMap.get(node) != null)
                mechModel.attachPoint(node, nodeMap.get(node));
        }

        for (int a=0; a<nFibres; a++)
        {
            int[] conn = mbd.connections.get(a); 
            for (int b=1; b<conn.length; b++)
            {
                Muscle fibre = new Muscle();
                fibre.setFirstPoint  ( bPoints.get(conn[b-1]) );
                fibre.setSecondPoint ( bPoints.get(conn[b]  ) );
                fibre.setRestLengthFromPoints();

                if (muscleMaterial instanceof AxialMuscleMaterial)
                {
                    AxialMuscleMaterial matCopy = (AxialMuscleMaterial)muscleMaterial.clone();
                    // TODO: opt and max length defined as optLength/length and maxLength/length in "mat"; I don't like this!
                    matCopy.setMaxLength(fibre.getLength() * matCopy.getMaxLength());  
                    matCopy.setOptLength(fibre.getLength() * matCopy.getOptLength());
                    fibre.setMaterial(matCopy);
                }
                else
                {
                    fibre.setMaterial(muscleMaterial.clone());
                }
                mb.addFibre (fibre);
            }
        }

        mb.setFibresActive(true);
        parentModel.addMuscleBundle(mb);
        
        return mb;
    }
    
    
    private enum ParseSection {None, Points, Connections};
    public static MuscleBundleData readMuscleBundle(String filename)
    {   
        MuscleBundleData mbd = new MuscleBundleData();

        try 
        {
            BufferedReader file = new BufferedReader(new FileReader(filename));
            String inLine = null;
            String[] inSplit = null;
            ParseSection parseSection = ParseSection.None;
            

            while (file.ready() == true)
            {
                inLine = file.readLine();
                inLine = inLine.trim();
                if (inLine.isEmpty() == true)
                {
                    parseSection = ParseSection.None; // a blank line marks the end of a section
                    continue;
                }
                if (inLine.startsWith("#") == true)
                    continue;   // comments are ignored
                if (inLine.contains("#") == true)
                {
                    inSplit = inLine.split("#");
                    inLine = inSplit[0]; // everything after first # is disregarded
                    inLine = inLine.trim();
                }
                
                // parse points
                if (inLine.startsWith("points") == true)
                {
                    parseSection = ParseSection.Points;
                    continue;
                }
                if (parseSection == ParseSection.Points)
                {
                    MusclePoint mp = new MusclePoint();

                    inSplit = inLine.split(",");
                    for (int a=0; a<inSplit.length; a++)
                        inSplit[a] = inSplit[a].trim();

                    mp.index = Integer.valueOf(inSplit[0]);
                    mp.loc = new Point3d(Double.valueOf(inSplit[1]), Double.valueOf(inSplit[2]), Double.valueOf(inSplit[3]));
                    // now parse any keywords
                    for (int a=4; a<inSplit.length; a++)
                    {
                        String[] kv = inSplit[a].split("=");
                        if      (kv[0].equalsIgnoreCase("type") == true)
                            mp.pointType = kv[1].trim();
                        else if (kv[0].equalsIgnoreCase("attach") == true)
                            mp.modelName = kv[1].trim();
                    }
                    mbd.musclePoints.add(mp);
                }
                
                // parse connectivity
                if (inLine.startsWith("connections") == true)
                {
                    parseSection = ParseSection.Connections;
                    continue;
                }
                if (parseSection == ParseSection.Connections)
                {
                    inSplit = inLine.split(",");
                    int[] conn = new int[inSplit.length];
                    for (int a=0; a<inSplit.length; a++)
                    {
                        conn[a] = Integer.valueOf(inSplit[a].trim());
                    }
                    mbd.connections.add(conn);
                }

            }
            file.close();
        }
        catch(Exception e) 
        {
            e.printStackTrace();
        }
        
        //mbd.reportMuscleStats(filename);
        
        return mbd;
    }
    
        
    public static void writeMuscleBundles(Iterable<MuscleBundle> muscleBundles, String dir)
    {
        for (MuscleBundle mb : muscleBundles)
        {
            String filename = dir + mb.getName() + ".txt";
            writeMuscleBundle(mb, filename);
        }
    }

    public static void writeMuscleBundle(MuscleBundle mb, String filename)
    {
        // this writes points, connectivity (fibres), point data (attachments, etc...), fibre data (muscle type...) 

        boolean writeFibreMaterial = false; // XXX: not thoroughly implemented
        boolean writePointAttachment = true;
        boolean writePointType = true;
        
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
            
            Date dNow = new Date();
            SimpleDateFormat ft = new SimpleDateFormat ("yyyy.MM.dd'_'HH:mm:ss zzz");
            file.printf("# Description of *%s* muscle bundle. Created on %s. \n\n", mb.getName(), ft.format(dNow));            
            
            ArrayList<Point> points = new ArrayList<Point>();

            for (Muscle m : mb.getFibres())
            {
                if ( points.contains(m.getFirstPoint()) == false )
                    points.add (m.getFirstPoint ());
                if ( points.contains(m.getSecondPoint()) == false )
                    points.add (m.getSecondPoint ());
            }

            //file.printf("points %d \n", points.size());
            file.printf("points\n");
            for (int i=0; i<points.size (); i++)
            {
                Point p = points.get (i);
                file.printf("%d, %f, %f, %f", i, p.getPosition().x, p.getPosition().y, p.getPosition().z);
                
                if (writePointType == true)
                {
                    // Point type
                    file.printf(", ");
                    if      (p instanceof FemNode3d)
                        file.printf("type=FemNode");
                    else if (p instanceof FemMarker)
                        file.printf("type=FemMarker");
                    else if (p instanceof FrameMarker)
                        file.printf("type=FrameMarker");
                    else if (p instanceof Particle)
                        file.printf("type=Particle");
                }
                
                if (writePointAttachment == true)
                {
                    
                    // Point attachment
                    file.printf(", ");
                    
                    if (p.isAttached() == true)
                    {
                        DynamicComponent dc = p.getAttachment().getMasters()[0];
                        if (dc instanceof FemNode3d)
                            file.printf("attach=%s", ((FemNode3d)dc).getGrandParent().getName());
                        else
                            file.printf("attach=%s", dc.getName());
                    }
                    else if (p.isDynamic() == false)
                    {
                        file.printf("attach=%s", "null"); // 
                    }
                    else
                    {
                        if      (p instanceof FemNode3d)
                            file.printf("attach=%s", ((FemNode3d)p).getGrandParent().getName() );
                        else if (p instanceof FemMarker)
                            file.printf("attach=%s", ((FemMarker)p).getGrandParent().getName() );
                        else if (p.getAttachment().numMasters() > 0)
                            file.printf("attach=%s", p.getAttachment().getMasters()[0].getName());
                        else
                            file.printf("attach=%s", "null");
                    }

                }
                file.println();
                
            }
            file.println();

            file.println("connections");
            int lastIndex = -1;
            for (Muscle m : mb.getFibres())
            {
                int i1 = points.indexOf(m.getFirstPoint());
                int i2 = points.indexOf(m.getSecondPoint());
                if (i1 != lastIndex)
                {
                    if (lastIndex == -1)
                        ;
                    else
                        file.printf( "\n" );
                    file.printf("%d, %d", i1, i2 );
                }
                else
                {
                    file.printf(", %d", i2 );
                }
                lastIndex = i2;
            }
            file.printf( "\n" );
            file.println();
            
            if (writeFibreMaterial == true)
            {
                file.println("material");

                for (Muscle m : mb.getFibres())
                {
                    AxialMaterial mat = m.getMaterial();
                    double length = m.getLength();
                    if (mat == null)
                        file.print ("null material");
                    else if (mat instanceof PeckAxialMuscle)
                    {
                        //AxialMuscleMaterial pMat = (AxialMuscleMaterial)mat;
                        PeckAxialMuscle pMat = (PeckAxialMuscle)mat;
                        file.printf("peckMuscle(%f, %f, %f, %f, %f, %f, %f),\n", pMat.getDamping(), pMat.getMaxForce(), pMat.getPassiveFraction(), pMat.getOptLength()/length, pMat.getMaxLength()/length, pMat.getTendonRatio(), pMat.getForceScaling() );
                    }
                    else if (mat instanceof ConstantAxialMuscle)
                    {
                        ConstantAxialMuscle pMat = (ConstantAxialMuscle)mat;
                        file.printf("constantMuscle(%f, %f, %f, %f, %f, %f, %f),\n", pMat.getDamping(), pMat.getMaxForce(), pMat.getPassiveFraction(), pMat.getOptLength()/length, pMat.getMaxLength()/length, pMat.getTendonRatio(), pMat.getForceScaling() );
                    }
                    // constant axial muscle
                    else
                    {
                        file.printf("%s: ", mat.getClass().toString());
                        for (PropertyInfo info : mat.getAllPropertyInfo() )
                            file.printf("%s=%s;", info.getName(), mat.getProperty(info.getName()).toString() );
                        file.println();
                    }
                }
            }
            file.close();
        }
        catch(Exception e)
        {
            System.out.printf("Error writing muscle bundle %s to %s.\n", mb.getName(), filename);
        }


    }
    
    
    public static void writeMuscleBundleData(MuscleBundleData mbd, String filename)
    {
        // this writes points, connectivity (fibres), point data (attachments, etc...), fibre data (muscle type...)
        
        boolean writePointAttachment = true;
        boolean writePointType = true;
        String name = "---";
        
        try
        {
            PrintWriter file = new PrintWriter(new BufferedWriter(new FileWriter(filename, false)));
            
            Date dNow = new Date();
            SimpleDateFormat ft = new SimpleDateFormat ("yyyy.MM.dd'_'HH:mm:ss zzz");
            file.printf("# Description of *%s* muscle bundle. Created on %s. \n\n", name, ft.format(dNow));            
            
            //file.printf("points %d \n", points.size());
            int nPoints = mbd.musclePoints.size();
            file.printf("points\n");
            for (int i=0; i<nPoints; i++)
            {
                Point3d p = mbd.musclePoints.get(i).loc;
                file.printf("%d, %f, %f, %f", i, p.x, p.y, p.z);
                
                if (writePointType == true)
                {
                    // Point type
                    file.printf(", ");
                    file.printf("type="+mbd.musclePoints.get(i).pointType);
                }
                
                if (writePointAttachment == true)
                {
                    // Point attachment
                    file.printf(", ");
                    file.printf("attach=%s", mbd.musclePoints.get(i).modelName);
                }
                file.println();
                
            }
            file.println();

            file.println("connections");
            for (int[] conn : mbd.connections)
            {
                for (int a=0; a<conn.length; a++)
                {
                    if (a != conn.length-1)
                        file.printf("%d, ", conn[a] );
                    else
                        file.printf("%d\n", conn[a] );
                }
                    
            }
            file.println();
            
            
            file.close();
        }
        catch(Exception e)
        {
            System.out.printf("Error writing muscle bundle %s to %s.\n", name, filename);
        }
    }
    
    
    public static ArrayList<Point3d> resampleOrderedPoints_linear(ArrayList<Point3d> inputPoints, double dx)
    {
        int nP0 = inputPoints.size(); // number of original points
        
        // define the distance function
        double[] x = new double[nP0];
        x[0] = 0.0;
        for (int a=1; a<nP0; a++)
            x[a] = x[a-1] + inputPoints.get(a).distance(inputPoints.get(a-1));
        
        
        ArrayList<Point3d> outputPoints = new ArrayList<>();
        outputPoints.add(inputPoints.get(0));
        double x_a = x[0]+dx;
        
        while (x_a < x[nP0-1])
        {
            int b=1;
            while ( (x_a > x[b]) && (b<nP0) )
                b++;
            
            Point3d pnt = new Point3d();
            double c = (x_a-x[b-1])/(x[b]-x[b-1]);
            pnt.x = inputPoints.get(b).x * c + inputPoints.get(b-1).x * (1.0 - c);
            pnt.y = inputPoints.get(b).y * c + inputPoints.get(b-1).y * (1.0 - c);
            pnt.z = inputPoints.get(b).z * c + inputPoints.get(b-1).z * (1.0 - c);
            outputPoints.add(pnt);
            
            x_a = x_a + dx;
        }
        outputPoints.add(inputPoints.get(nP0-1));
        int nP1 = outputPoints.size();
        
        if (outputPoints.get(nP1-1).distance(outputPoints.get(nP1-2)) < dx/10.0)
            outputPoints.remove(nP1-2);
        
        return outputPoints;
    }
    
    public static MuscleBundleData resampleMuscleBundlePoints(MuscleBundleData mbd, double dx)
    {
        MuscleBundleData mbdNew = new MuscleBundleData();
        for (int[] conn : mbd.connections)
        {
            int nPointsOrig = conn.length;
            ArrayList<Point3d> origPoints = new ArrayList<>();
            for (int idx : conn)
                origPoints.add(mbd.musclePoints.get(idx).loc);
            
            ArrayList<Point3d> newPoints = resampleOrderedPoints_linear(origPoints, dx);
            int nP = newPoints.size();
            
            // add the point to mbd, get its index, and
            int[] newConn = new int[nP];
            for (int a=0; a<nP; a++)
            {
                Point3d pnt = newPoints.get(a);
                int iPair = 0;
                double dPair = Double.POSITIVE_INFINITY;
                for (int b=0; b<nPointsOrig; b++)
                {
                    if (pnt.distance(origPoints.get(b)) < dPair)
                    {
                        dPair = pnt.distance(origPoints.get(b));
                        iPair = conn[b];
                    }
                }
                MusclePoint mp = new MusclePoint();
                mp.loc = pnt;
                mp.modelName = mbd.musclePoints.get(iPair).modelName;
                mp.pointType = mbd.musclePoints.get(iPair).pointType;
                mbdNew.musclePoints.add(mp);
                newConn[a] = mbdNew.musclePoints.indexOf(mp);
            }
            mbdNew.connections.add(newConn);
        }
        return mbdNew;
    }
    
}

class MuscleBundleData
{
    ArrayList<MusclePoint> musclePoints = new ArrayList<>();
    ArrayList<int[]> connections = new ArrayList<>();
    
    public void mirrorPoints(int axis)
    {
        for (MusclePoint p : musclePoints)
        {
            p.loc.set(axis, -p.loc.get(axis));
        }
    }
    
    public void reportMuscleStats(String ID)
    {
        int N = connections.size();
        
        System.out.printf("Muscle %s \n", ID);
        
        double avglen = 0.0;
        for (int a=0; a<connections.size(); a++)
        {
            int[] conn = connections.get(a);
            double len = 0.0;
            for (int i=1; i<conn.length; i++)
            {
                len = len + musclePoints.get(conn[i-1]).loc.distance( musclePoints.get(conn[i]).loc );
            }
            
            avglen = avglen + len;
            System.out.printf("fiber %d: len = %f \t", a, len);
        }
        avglen = avglen/(double)N;
        
        System.out.println();
        System.out.printf("%d fibers, avg len = %f \n\n", N, avglen);   
    }
    
    public static ArrayList<Point3d> resampleOrderedPoints_nurbs(ArrayList<Point3d> inputPoints, double dx)
    {
//        Vector4d[] wPoints = new Vector4d[nPoints];
//        for (int a=0; a<nPointsOrig; a++)
//            wPoints[a] = new Vector4d( origPoints.get(a).x, origPoints.get(a).y, origPoints.get(a).z, 1.0);
//        NURBSCurve3d spline = new NURBSCurve3d(degree, NURBSCurve3d.OPEN, wPoints, null);
//
//        double uMin = spline.findPoint(origPoints.get(0),-1,2);
//        double uMax = spline.findPoint(origPoints.get(nPointsOrig-1),-1,2);
//        double du = (uMax-uMin)/((double)nSegments);
//        
//        //
//        ArrayList<Point3d> newPoints = new ArrayList<>();
//        for (int a=0; a<nPoints; a++)
//        {
//            Point3d p = new Point3d();
//            spline.eval(p, du*((double)a) + uMin );
//            newPoints.add(p);
//        }
        
        return null;
        
        // my resampling points are not evenly spaced when using the spline, they depend on the original points
        // length should be an integration along the curve, and not just piecewise, then resample for evenly spaced ds rather than du
        // also note that this spline is *not* constrained to pass through each control point (though I'd prefer otherwise)
        // how can I design an evenly spaced function s/S = u/U
        
     // each strand will be resampled with nPoints??
        // better to use a dx --> dx/length = du/(uMax-uMin)
        // how can I accurately calc the length?
        // or just use a linear resampling...which would be fine!
    }
}

class MusclePoint
{
    int index;
    Point3d loc;
    String modelName = null;
    String pointType = null;
}
