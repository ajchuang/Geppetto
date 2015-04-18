using System;
using System.Diagnostics;

class vec {

    int m_x;
    int m_y;
    int m_z;
    double m_length;

    public vec (int x, int y, int z) {
        m_x = x;
        m_y = y;
        m_z = z;
        
        m_length = Math.Sqrt (x*x + y*y + z*z);
    }
    
    public int x () {return m_x;}
    public int y () {return m_y;}
    public int z () {return m_z;}
    public double length () {return m_length; }
    
    /* calculate the dot */
    public int dot (vec v) {
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
        Debug.WriteLine ("angle: " + cos_ang);
        return cos_ang;
    }
};
