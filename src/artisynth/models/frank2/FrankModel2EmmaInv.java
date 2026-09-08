package artisynth.models.frank2;

import java.awt.Color;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import maspack.interpolation.Interpolation;
import maspack.interpolation.NumericList;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.util.ReaderTokenizer;
import artisynth.core.inverse.DampingTerm;
import artisynth.core.inverse.L2RegularizationTerm;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.ExcitationComponent;
import artisynth.core.mechmodels.Marker;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MuscleExciter;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.AffineNumericInputProbe;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.workspace.DriverInterface;


public class FrankModel2EmmaInv extends FrankModel2Emma {

    ArrayList<MotionTargetComponent> motionTargets = null;
    AffineNumericInputProbe targetProbe = null;
    TrackingController tracker = null;
    
    boolean adjustProbePos = true; // shift the initial probe data to match the tongue
    boolean useTimeOffset = true;  // allow use of inverseOffset or not
    double inverseOffset = 0.1;     // seconds to offset probe to allow for initial alignment

    @Override
    public void build (String[] args) throws IOException 
    {
        super.build (args);

        removeAllInputProbes();
        mechModel.remove (falseTargets);
        removeAllOutputProbes();
        targetProbe = new AffineNumericInputProbe ();
        configureInverseController(mySyllables);
        configureInverseProbes (mySyllables);

    }

    @Override
    public void attach (DriverInterface driver) {
        super.attach (driver);

        for (MotionTargetComponent m : motionTargets) {
            RenderProps.setPointRadius ((Renderable)m, 0.002);
            RenderProps.setPointColor ((Renderable)m, Color.MAGENTA);
        }
    }

    protected void addExcitersToInverse (TrackingController trackingController) 
    {
        // Add exciters to the model to be used by inverse Modeler
        // TODO: prev version used sp, ip as the JO exciters
        ArrayList<MuscleExciter> allExciters = getAllExciters();
        MuscleExciter mex_jo = FrankActivations.makeJawOpenExciter(allExciters);
        MuscleExciter mex_jc = FrankActivations.makeJawCloseExciter(allExciters);
        externalExciters.add(mex_jo);
        externalExciters.add(mex_jc);

        // add jaw exciters
        trackingController.addExciter(mex_jo);
        trackingController.addExciter(mex_jc);

        // tongue exciters
        for (MuscleExciter ex : tongueExciters ) 
            trackingController.addExciter(ex);
        
        // palate exciters
        for (MuscleExciter ex :palateExciters ) 
            trackingController.addExciter(ex);
        
        // add face exciters??
    }

    protected void configureInverseController(String syll) 
    {
        tracker = new TrackingController (mechModel, "Emma Inverse");

        for (Marker m : inverseMarkers) {
            tracker.addMotionTarget (m);
        }

        //tracker.setManaged(false); XXX: TrackingController was refactored...is this no longer needed?
        //tracker.setEnabled(true);
        addExcitersToInverse (tracker);

        motionTargets = tracker.getMotionTargetTerm ().getTargets ();
        
        tracker.addL2RegularizationTerm (0.05);
        //DampingTerm trackerDampingTerm = new DampingTerm (tracker); // orig
        tracker.addDampingTerm (0.05);


        tracker.setExcitationBounds(0,1); 
        tracker.setMaxExcitationJump (0.01); // orig 0.1
        addController (tracker);
        configureOutputProbes(tracker, syll);


        //tracker.createInverseControlPanel ();

    }

    private void configureOutputProbes(TrackingController track, String syll) {

        NumericOutputProbe outProbe = new NumericOutputProbe();
        outProbe.setName("Target Output Positions");
        outProbe.setAttachedFileName(filenameTargetOut);

        ArrayList<Property> props = new ArrayList<Property>();

        motionTargets = tracker.getMotionTargetTerm ().getTargets ();

        for (ModelComponent target : motionTargets) {
            if (target instanceof Point) {
                props.add(target.getProperty("position"));
            }
        }
        outProbe.setModel(track.getMech());
        outProbe.setOutputProperties(props.toArray(new Property[props.size()]));
        addOutputProbe(outProbe);


        outProbe = new NumericOutputProbe();
        outProbe.setName("Activation Parameters Output");
        outProbe.setAttachedFileName(filenameActivOut);
        props = new ArrayList<Property>();

        for (ExcitationComponent mex : track.getExciters()) {
            if (mex instanceof MuscleExciter) {
                props.add(((MuscleExciter)mex).getProperty("excitation"));
            }
        }
        outProbe.setModel(track.getMech());
        outProbe.setOutputProperties(props.toArray(new Property[props.size()]));
        addOutputProbe(outProbe);

    }

    //   @Override
    //   public StepAdjustment advance (double t0, double t1, int flags) {
    //   
    //      // points, markers
    //      VectorNd data = targetProbe.getData (t0);
    //      System.out.print("Probe:  ");
    //      for (int i=0; i<data.size (); i++) {
    //         System.out.print (data.get (i) + " ");
    //      }
    //      System.out.println ();
    //      
    //      System.out.print("Target: ");
    //      double pos[] = new double[3];
    //      for (MotionTargetComponent mtc : motionTargets) {
    //         int nvals = mtc.getPosState (pos, 0);
    //         for (int i=0; i<nvals; i++) {
    //            System.out.print (pos[i] + " ");
    //         }
    //      }
    //      System.out.println();
    //      
    //      return super.advance (t0, t1, flags);
    //   }

    protected void configureInverseProbes (String syll) 
    {
        targetProbe.setName ("Target Input Positions (Inverse)");
        targetProbe.setAttachedFileName (filenameTargetIn);

        ArrayList<Property> props = new ArrayList<Property> ();
        for (MotionTargetComponent target : motionTargets) {
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
        //targetProbe.setInterpolationOrder (Interpolation.Order.Linear);
        targetProbe.setInterpolationOrder (Interpolation.Order.Cubic); // seems to help

        // transform probe
        try 
        {
            ReaderTokenizer rtok = new ReaderTokenizer (new FileReader (filenameAlign));
            double[] vals = new double[12];
            int nvals = rtok.scanNumbers (vals, 12);
            if (nvals == 12) {
                AffineTransform3d probeTrans = new AffineTransform3d(vals[0],vals[1],vals[2],vals[9],
                    vals[3], vals[4], vals[5], vals[10], vals[6], vals[7], vals[8], vals[11]);
                targetProbe.setTransform (probeTrans);
            }
        } catch (IOException e) 
        {
            //e.printStackTrace();
            System.out.println("Inverse controller warning: probe alignment file not found. Assuming no transform.");
        }
        
        if (adjustProbePos == true)
        {
            //targetProbe.getNumericList()
            double[][] pdata = targetProbe.getValues();
            double[] deltas = new double[inverseMarkers.size()*3];
            
            AffineTransform3d trans = targetProbe.getTransform ();
            trans.invert();
            
            int idx = 0;
            for (Marker m : inverseMarkers) 
            {
                Point3d ip = new Point3d(m.getPosition ());
                ip.transform (trans);
                
                deltas[idx] = ip.x - pdata[0][idx+1]; idx++;
                deltas[idx] = ip.y - pdata[0][idx+1]; idx++;
                deltas[idx] = ip.z - pdata[0][idx+1]; idx++;
            }
            for (int a=0; a<pdata.length; a++)
            {
                for (int b=1; b<pdata[a].length; b++)
                {
                    pdata[a][b] = pdata[a][b] + deltas[b-1]*0.5; //FIXME: scaling by 0.5 is a hack...
                }
            }
            
            targetProbe.setValues(pdata);
            targetProbe.getNumericList().getFirst();
            
        }

        if (useTimeOffset == true)
        {
            // shift time by offset
            NumericList nlist = targetProbe.getNumericList ();
            nlist.shiftTime(inverseOffset);

            // add artificial starting point
            double[] currentLocs = new double[inverseMarkers.size()*3];
            int idx = 0;
            AffineTransform3d trans = targetProbe.getTransform ();
            trans.invert();
            for (Marker m : inverseMarkers) {
                Point3d backPos = new Point3d(m.getPosition ());
                backPos.transform (trans);
                currentLocs[idx++] = backPos.x;
                currentLocs[idx++] = backPos.y;
                currentLocs[idx++] = backPos.z;
            }
            targetProbe.addData (0, currentLocs);
            targetProbe.addData (inverseOffset/2, currentLocs);
        }

        addInputProbe (targetProbe);
        targetProbe.initialize (0);    // move points to correct place


    }   

}
