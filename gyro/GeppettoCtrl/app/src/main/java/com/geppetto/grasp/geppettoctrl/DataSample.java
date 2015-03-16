package com.geppetto.grasp.geppettoctrl;

/**
 * Created by alfredhuang on 3/15/15.
 */
public class DataSample {

    public long m_ts;
    public float m_x;
    public float m_y;
    public float m_z;

    public DataSample (long ts, float x, float y, float z) {
        m_x  = x;
        m_y  = y;
        m_z  = z;
        m_ts = ts;
    }
}
