package artisynth.models.frank2;

import java.awt.Color;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;
import maspack.util.ReaderTokenizer;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.femmodels.MuscleBundle;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.Marker;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.probes.AffineNumericInputProbe;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;


public class FrankModel2Emma extends FrankModel2 {

    //protected static LinkedHashMap<String,String> fileLabels = null;
    // bunch of syllable pairs and corresponding files
    static String[] sylls = { "bali", "basa", "baspa", "baya", "dela" };
    
    /*/ switch between using synthetic vs real Emma data (inv with real Emma not too robust)
    boolean isSyntheticEmma = false;
    public String mySyllables = "bali";
    String root = ArtisynthPath.getSrcRelativePath (FrankModel2Emma.class, "emma/");
    /*/
    boolean isSyntheticEmma = true;
    public String mySyllables = "synthetic_a_jtp"; //
    String root = ArtisynthPath.getSrcRelativePath (FrankModel2Emma.class, "emma_synth/");
    //*/
    
    String filenameTargetIn  = root + mySyllables + "_target_in.txt";
    String filenameAlign     = root + mySyllables + "_align.txt";
    String filenameMarkers   = root + mySyllables + "_tongue_markers.txt";
    String filenameTargetOut = root + mySyllables + "_target_out.txt";
    String filenameActivOut  = root + mySyllables + "_activation_out.txt";
    
//    static 
//    {
//        fileLabels = new LinkedHashMap<String,String> ();
//        
//        for (String syll : sylls) 
//        {
//            fileLabels.put ("targets_" + syll,       root + syll + "_target_in.txt");
//            fileLabels.put ("alignment_" + syll,     root + syll + "_align.txt");
//            fileLabels.put ("markers_" + syll,       root + syll + "_tongue_markers.txt");
//            fileLabels.put ("target_out" + syll,     root + syll + "_target_out.txt");
//            fileLabels.put ("activation_out" + syll, root + syll + "_activation_out.txt");
//        }
//    }

    

    public double stopTime = 5.5; // gets over-ridden
    public double NumberofFrames = 100; // number of waypoints

    PointList<Point> falseTargets = null;
    ArrayList<Marker> inverseMarkers = null;

    @Override
    public void build (String[] args) throws IOException 
    {
        super.excitersCombineLR = true; // this inverse model expects bilateral muscles
        
        super.build (args);

        //this.setMaxStepSize(0.005); // 0.1 ??

        removeAllInputProbes();
        removeAllOutputProbes();

        
        if (isSyntheticEmma == true)
        {
            inverseMarkers = super.createSynthEmmaPoints();
        }
        else
        {
            // add 5 markers
            inverseMarkers = new ArrayList<Marker>();
            inverseMarkers.addAll(addIncisorMarkers());
            inverseMarkers.addAll(addTongueMarkers(mySyllables));
        }
        
        // create 5 generic targets
        falseTargets = createTargets(inverseMarkers.size());  // orig 5: 4 for tongue, 1 for jaw
        mechModel.add (falseTargets);

        NumericInputProbe targetProbe = configureInputProbes (mySyllables);
        
        stopTime = targetProbe.getNumericList().getLast().t;
        addWayPoints (this, stopTime, (stopTime/NumberofFrames));
    }

    
    @Override
    public void attach (DriverInterface driver) {
        super.attach (driver);

        // set up render properties
        RenderProps.setPointStyle (falseTargets, PointStyle.SPHERE);
        RenderProps.setPointColor (falseTargets, Color.RED);
        RenderProps.setPointRadius (falseTargets, 0.002);

        for (Marker m : inverseMarkers) {
            RenderProps.setPointStyle (m, PointStyle.SPHERE);
            RenderProps.setPointColor (m, Color.BLUE);
            RenderProps.setPointRadius (m, 0.002);
        }

    }

    protected static PointList<Point> createTargets(int n) 
    {
        PointList<Point> targets = new PointList<Point>(Point.class, "EMMA targets");
        for (int i=0; i<n; ++i) {
            targets.add (new Point());
        }
        return targets;
    }

    protected NumericInputProbe configureInputProbes (String syll) {
        AffineNumericInputProbe targetProbe = new AffineNumericInputProbe ();

        targetProbe.setName ("Target Input Positions");
        targetProbe.setAttachedFileName ( filenameTargetIn );

        ArrayList<Property> props = new ArrayList<Property> ();
        for (MotionTargetComponent target : falseTargets) {
            if (target instanceof Point) {
                props.add (target.getProperty ("position"));
            }
        }
        targetProbe.setModel (mechModel);
        targetProbe.setInputProperties (props.toArray (new Property[props.size ()]));
        try {
            targetProbe.load ();
        }
        catch (IOException e) {
            e.printStackTrace ();
        }
        addInputProbe (targetProbe);

        // transform probe
        try 
        {
            ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (filenameAlign));
            double[] vals = new double[12];
            int nvals = rtok.scanNumbers (vals, 12);
            if (nvals == 12) 
            {
                AffineTransform3d probeTrans = new AffineTransform3d(vals[0],vals[1],vals[2],vals[9],
                    vals[3], vals[4], vals[5], vals[10], vals[6], vals[7], vals[8], vals[11]);
                targetProbe.setTransform (probeTrans);
            }
            rtok.close ();
        } 
        catch (IOException e) 
        {
            //e.printStackTrace();
            System.out.println("Inverse controller warning: probe alignment file not found. Assuming no transform.");
        }

        targetProbe.initialize (0);    // move points to correct place
        targetProbe.apply (0);
        
        return targetProbe;
    }

    public ArrayList<Marker> addIncisorMarkers () 
    {
        FrameMarker lowerIncisor = new FrameMarker ("lowerIncisor");
        lowerIncisor.setFrame (jaw);
        lowerIncisor.setLocation (new Point3d (0.0561215, 0, 0.0886421));
        //lowerIncisor.setLocation (new Point3d (-0.0433748, 0, 0.00621351));
        mechModel.addFrameMarker (lowerIncisor);
        
        ArrayList<Marker>  markers = new ArrayList<>();
        markers.add(lowerIncisor);
        return markers;
    }

    public ArrayList<Marker> addTongueMarkers(String syll) 
    {
        ArrayList<Marker>  markers = new ArrayList<>();
        
        // add four points on tongue
        try {
            ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (filenameMarkers));
            double vals[] = new double[8];
            int ivals[] = new int[8];

            // 8 node indices, 8 coordinates
            for (int i=0; i<4; ++i) {
                int nivals = rtok.scanIntegers (ivals, 8);
                int nvals = rtok.scanNumbers (vals, 8);
                if (nivals == 8 && nvals == 8) {
                    Point3d pnt = new Point3d ();
                    Point3d newloc = new Point3d ();

                    // compute world location
                    for (int j = 0; j < nivals; j++) {
                        pnt.scaledAdd (vals[j], tongue.getNode (ivals[j]).getPosition ());
                    }

                    FemElement3dBase elem = tongue.findNearestElement(newloc, pnt);
                    FemMarker m = new FemMarker (elem, newloc);
                    m.setName ("tongueMarker" + i);
                    markers.add(m);
                    tongue.addMarker (m, elem);
                }
            }
            rtok.close ();
        }
        catch (IOException e) {
            e.printStackTrace();
        } 
        
        return markers;
    }

    public void addWayPoints (RootModel root, double duration, double waypointstep) 
    {
        removeAllWayPoints();
        for (int i = 1; i < duration / waypointstep; i++) 
        {
            root.addWayPoint(i * waypointstep);
        }
        root.addBreakPoint(duration);
    }
    
}
