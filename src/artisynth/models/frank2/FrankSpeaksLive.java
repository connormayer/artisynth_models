package artisynth.models.frank2;

import java.io.IOException;

import maspack.geometry.PolylineMesh;

import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.fluid1d.csa.AreaFunctionMonitor;
import artisynth.models.jassRendering.JassMonitor;


public class FrankSpeaksLive extends FrankModel2  
{
    AreaFunctionMonitor afMonitor;
    JassMonitor jMonitor;
    
    public void build (String[] args) throws IOException 
    {
        super.build(args);
        
     // --- define the area function monitor --- //
        afMonitor = new AreaFunctionMonitor();
        afMonitor.setGeometry(airway.getSurfaceMesh());
        PolylineMesh plm = (PolylineMesh)centerlineSkin.getMesh();
        afMonitor.setCenterline(plm.getLines().get(0));
        afMonitor.setSliceEachStep(false);
        //afMonitor.setUpdateCenterlineByCentroids(true);
        afMonitor.setPrintToScreen(false);
        afMonitor.initialize();
        afMonitor.getPlot().getAxis(0).setRange(0.0,  0.2, 0.0, 0.05);
        this.addMonitor(afMonitor);
        
        // --- define the jass monitor --- //
        jMonitor = new JassMonitor(this);
        this.addMonitor(jMonitor);
        jMonitor.setAreas(afMonitor.getAreas());
        jMonitor.setLength(afMonitor.getDists()[afMonitor.getNumPoints()-1]);
        jMonitor.setWriteJassFile(outputDir + "jassData.txt");
        jMonitor.setWriteWavFile(outputDir + "jassSound.wav");
        jMonitor.initialize();
    }
    
    @Override
    public StepAdjustment advance( double t0, double t1, int flags) 
    {
        jMonitor.setAreas(afMonitor.getAreas());
        jMonitor.setLength(afMonitor.getDists()[afMonitor.getNumPoints()-1]);
        
        return super.advance(t0, t1, flags);
    }
}
