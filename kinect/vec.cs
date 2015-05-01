using System;
using System.Diagnostics;

class vec {

    double m_x;
    double m_y;
    double m_z;
    double m_length;

    public vec (double x, double y, double z) {
        m_x = x;
        m_y = y;
        m_z = z;
        
        m_length = Math.Sqrt (x*x + y*y + z*z);
    }
    
    public double x () {return m_x;}
    public double y () {return m_y;}
    public double z () {return m_z;}
    
    public void set_x (double x) { m_x = x;}
    public void set_y (double y) { m_y = y;}
    public void set_z (double z) { m_z = z;}
    
    public double length () {return m_length; }
    
    /* calculate the dot */
    public double dot (vec v) {
        return 
            m_x * v.x () + 
            m_y * v.y () +
            m_z * v.z ();
    }
    
    public double angle (vec v) {
        
        if (v.length () < 0.01 || m_length < 0.01) {
            Debug.WriteLine ("Error: zero length vector");
            return 0.0;
        }
        
        double cos_ang = Math.Acos (dot (v) / (v.length() * m_length));
        //Debug.WriteLine ("angle: " + cos_ang);
        return cos_ang;
    }
};
