using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class DataPoint
    {
        public double m_r1;
        public double m_r2;
        public double m_r3;
        public double m_r4;
        public double m_r5;
        public double m_r6;
        public double m_r7;

        public double m_l1;
        public double m_l2;
        public double m_l3;
        public double m_l4;
        public double m_l5;
        public double m_l6;
        public double m_l7;

        public DataPoint()
        {
            m_r1 = 0.0;
            m_r2 = 0.0;
            m_r3 = 0.0;
            m_r4 = 0.0;
            m_r5 = 0.0;
            m_r6 = 0.0;
            m_r7 = 0.0;

            m_l1 = 0.0;
            m_l2 = 0.0;
            m_l3 = 0.0;
            m_l4 = 0.0;
            m_l5 = 0.0;
            m_l6 = 0.0;
            m_l7 = 0.0;
        }

        public DataPoint(
            double r1, double r2, double r3, double r4, double r5, double r6, double r7,
            double l1, double l2, double l3, double l4, double l5, double l6, double l7)
        {
            m_r1 = r1;
            m_r2 = r2;
            m_r3 = r3;
            m_r4 = r4;
            m_r5 = r5;
            m_r6 = r6;
            m_r7 = r7;

            m_l1 = l1;
            m_l2 = l2;
            m_l3 = l3;
            m_l4 = l4;
            m_l5 = l5;
            m_l6 = l6;
            m_l7 = l7;
        }

        public void add (DataPoint dp)
        {
            m_r1 += dp.m_r1;
            m_r2 += dp.m_r2;
            m_r3 += dp.m_r3;
            m_r4 += dp.m_r4;
            m_r5 += dp.m_r5;
            m_r6 += dp.m_r6;
            m_r7 += dp.m_r7;

            m_l1 += dp.m_l1;
            m_l2 += dp.m_l2;
            m_l3 += dp.m_l3;
            m_l4 += dp.m_l4;
            m_l5 += dp.m_l5;
            m_l6 += dp.m_l6;
            m_l7 += dp.m_l7;
        }

        public void divide(double x)
        {
            m_r1 /= x;
            m_r2 /= x;
            m_r3 /= x;
            m_r4 /= x;
            m_r5 /= x;
            m_r6 /= x;
            m_r7 /= x;

            m_l1 /= x;
            m_l2 /= x;
            m_l3 /= x;
            m_l4 /= x;
            m_l5 /= x;
            m_l6 /= x;
            m_l7 /= x;

        }
    }
}
