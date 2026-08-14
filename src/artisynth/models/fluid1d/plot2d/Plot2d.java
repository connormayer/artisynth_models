package artisynth.models.fluid1d.plot2d;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Toolkit;
import java.util.ArrayList;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JPanel;

// maybe just extend jframe??
public class Plot2d 
{
    String name = "Plot2d";
    JFrame plotFrame = new JFrame();
    ArrayList<Plot2dAxes> axes = new ArrayList<Plot2dAxes>();

    public Plot2d()
    {
    }
    
    public Plot2d(String name)
    {
        this.name = name;
    }
    
    public void buildPlot()
    {
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        int width  = (int)(0.4*screenSize.getWidth());
        int height = (int)(0.9*screenSize.getHeight());

        buildPlot((int)(screenSize.getWidth()-width), 0, width, height);
    }
    
    public void buildPlot(int x0, int y0, int width, int height)
    {
        JPanel layout = new JPanel();
        layout.setLayout (new GridLayout(axes.size (),1));
        for (Plot2dAxes ax : axes)
        {
            JPanel bpanel = new JPanel();
            bpanel.setBorder (BorderFactory.createTitledBorder (BorderFactory.createEtchedBorder (), ax.getName() ));
            bpanel.add (ax);
            bpanel.setLayout (new GridLayout(1,1));
            layout.add (bpanel);
        }
        
        //layout.add (pax);
        plotFrame.add(layout);
        plotFrame.setVisible (true);
        plotFrame.setName (this.name);
        plotFrame.setTitle ("Plot2d");

        plotFrame.setBounds (x0, y0, width, height);
    }

    public void update()
    {
        plotFrame.repaint();
    }

    public void addAxis(Plot2dAxes axis)
    {
        this.axes.add (axis);
    }

    public Plot2dAxes getAxis(int i)
    {
        return axes.get(i);
    }
    
    public Plot2dAxes getAxis(String name)
    {
        for (Plot2dAxes ax : axes)
        {
            if (ax.getName() == name)
                return ax;
        }
        return null;
    }
    
    public int getNumAxes()
    {
        return axes.size();
    }

    public Plot2dAxes createAxis(String name)
    {
        Plot2dAxes ax = new Plot2dAxes();
        ax.setName (name);
        addAxis(ax);
        return ax;
    }

}
