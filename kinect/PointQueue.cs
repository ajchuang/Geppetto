using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class PointQueue
    {
        Queue<DataPoint> m_pointQueue;
        int M_PTRS_IN_Q = 10;

        public PointQueue()
        {
            m_pointQueue = new Queue<DataPoint>();
        }

        public void addPoint (DataPoint dp)
        {
            if (m_pointQueue.Count < M_PTRS_IN_Q) {
                m_pointQueue.Enqueue (dp);
            } else {
                m_pointQueue.Dequeue();
                m_pointQueue.Enqueue(dp);
            }
        }

        /* TODO: need to add the pointer filtering */
        public DataPoint Average ()
        {
            DataPoint dp = new DataPoint();
            int cnt = m_pointQueue.Count;

            while (m_pointQueue.Count > 0)
            {
                dp.add (m_pointQueue.Dequeue ());
            }

            dp.divide((double)cnt);
            return dp;
        }
    }
}
