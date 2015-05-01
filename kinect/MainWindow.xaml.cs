/*
    1.  shoulder_pan:   arm - spine (0: front)
    2.  shoulder_lift:  arm - shoulder (0: front, 1: down)
    3.  upper_arm_roll: (calculated)
    4.  elbow_flex:     arm-elbow (0: total straight)
    5.  forearm row:    no way
    6.  wrist flex:     no  
    7.  wrist roll:     no
 */
namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System;
    using System.Text;
    using System.IO;
    using System.Net; 
    using System.Net.Sockets;
    using System.Windows;
    using System.Windows.Media;
    using System.Diagnostics;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        int m_sample = 0;

        PointQueue m_queue;

        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        
        /* networking */
        NetworkStream m_networkStream;
        TcpClient m_tcpClient;

        /* config */
        string m_hostName = "128.59.19.233";
        int m_port = 4009;
        bool m_testing = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            m_queue = new PointQueue();
            InitializeComponent ();
            InitializeConnection ();
        }
        
        void InitializeConnection () 
        {
            if (m_testing)
                return;

            try {
                m_tcpClient = new TcpClient ();
                m_tcpClient.Connect (m_hostName, m_port);
                m_networkStream = m_tcpClient.GetStream();
                Trace.WriteLine ("connected");
            } catch {
                Trace.WriteLine ("Oooops");
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                            this.reportPos(skel);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        // @lfred: TODO send information to the proxy
        private void reportPos (Skeleton sk) {

            
            //dumpPositons (sk);

            double r1 = calculateRightArmPam(sk);
            double r2 = calculateRightArmLift(sk);
            double r3 = calculateRightArmRoll(sk);
            double r4 = calculateRightforeArmLift(sk);
            double r5 = 0.0;
            double r6 = 0.0;
            double r7 = 0.0;
            double l1 = calculateLeftArmPam(sk);
            double l2 = calculateLeftArmLift(sk);
            double l3 = calculateLeftArmRoll(sk);
            double l4 = calculateLeftforeArmLift(sk);
            double l5 = 0.0;
            double l6 = 0.0;
            double l7 = 0.0;
            
            if (r1 == double.NegativeInfinity || r2 == double.NegativeInfinity ||
                r3 == double.NegativeInfinity || r4 == double.NegativeInfinity ||
                l1 == double.NegativeInfinity || l2 == double.NegativeInfinity ||
                l3 == double.NegativeInfinity || l4 == double.NegativeInfinity) {
            
                Trace.WriteLine ("[Warning] Untracked point");
                return;
            }


            
            m_queue.addPoint(new DataPoint(r1, r2, r3, r4, r5, r6, r7, l1, l2, l3, l4, l5, l6, l7));

            /* @lfred: to reduce the number of points */
            if (m_sample < 50)
            {
                m_sample++;
                return;
            }
            else
            {
                m_sample = 0;
            }

            DataPoint dp = m_queue.Average();
            
            string out_str = 
                "GO " +
                String.Format ("{0:0.##}", dp.m_r1) + " " + String.Format ("{0:0.##}", dp.m_r2) + " " + 
                String.Format ("{0:0.##}", dp.m_r3) + " " + String.Format ("{0:0.##}", dp.m_r4) + " " + 
                String.Format ("{0:0.##}", dp.m_r5) + " " + String.Format ("{0:0.##}", dp.m_r6) + " " +
                String.Format ("{0:0.##}", dp.m_r7) + " " + String.Format ("{0:0.##}", dp.m_l1) + " " + 
                String.Format ("{0:0.##}", dp.m_l2) + " " + String.Format ("{0:0.##}", dp.m_l3) + " " + 
                String.Format ("{0:0.##}", dp.m_l4) + " " + String.Format ("{0:0.##}", dp.m_l5) + " " + 
                String.Format ("{0:0.##}", dp.m_l6) + " " + String.Format ("{0:0.##}", dp.m_l7) + " ";
            
            Trace.WriteLine(out_str);
            
            if (m_testing)
                return;
            
            Byte[] myBytes = Encoding.ASCII.GetBytes (out_str);
            
            try
            {
                m_networkStream.Write(myBytes, 0, myBytes.Length);
                m_networkStream.Flush();
            }
            catch
            {
                return;
            }
            
        }
        
        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }
        
        private vec getPointVect (Skeleton skeleton, JointType j, JointType r) 
        {
            // check if the point is tracked
            if (skeleton.Joints[j].TrackingState == JointTrackingState.Tracked && 
                skeleton.Joints[r].TrackingState == JointTrackingState.Tracked) 
            {
                return null;
            }

            SkeletonPoint sp    = skeleton.Joints[j].Position;
            SkeletonPoint sp_r  = skeleton.Joints[r].Position;
            return new vec (sp.X - sp_r.X, sp.Y - sp_r.Y, sp.Z - sp_r.Z);
        }
        
        private string dumpPoint (Skeleton skeleton, JointType j) 
        {
            SkeletonPoint sp = skeleton.Joints[j].Position;
            string s = "(" + sp.X + ":" + sp.Y + ":" + sp.Z + ")";
            return s;
        }
        
        private double calculateLeftArmPam (Skeleton sk)
        {
            double lp;
            vec rvc = new vec(1.0, 0.0, 0.0);
            vec arm = getPointVect (sk, JointType.ElbowLeft, JointType.ShoulderLeft);
            
            if (arm == null)
                return double.NegativeInfinity;
            
            rvc.set_y (0.0);
            arm.set_y (0.0);

            lp = rvc.angle(arm) - (Math.PI)/2; 
            // Debug.WriteLine("langle: " + lp);
            return lp;
        }
        
        private double calculateLeftArmLift (Skeleton sk)
        {
            double lal;
            vec x = new vec(0.0, 1.0, 0.0);
            vec arm = getPointVect(sk, JointType.ShoulderLeft, JointType.ElbowLeft);
            
            if (arm == null)
                return double.NegativeInfinity;
            
            x.set_x (0.0);
            arm.set_x (0.0);
            lal = (Math.PI)/2 - x.angle(arm);

            //Debug.WriteLine("langle: " + lal);
            return lal;
        }

        private double calculateLeftArmRoll(Skeleton sk)
        {
            double lar;
            vec re = new vec(1.0, 0.0, 0.0);
            vec arm = getPointVect(sk, JointType.WristLeft, JointType.ElbowLeft);

            if (arm == null)
                return double.NegativeInfinity;
            
            re.set_z(0.0);
            arm.set_z(0.0);

            lar = (Math.PI)/2 - re.angle(arm); 


            //Debug.WriteLine("rangle: " + lar);
            return lar;
        }

        private double calculateLeftforeArmLift(Skeleton sk)
        {
            double lfa;
            vec rvc = getPointVect(sk, JointType.WristLeft, JointType.ElbowLeft);
            vec arm = getPointVect(sk, JointType.ElbowLeft, JointType.ShoulderLeft);

            if (arm == null || rvc == null)
                return double.NegativeInfinity;
            
            lfa = -rvc.angle(arm);
            //Debug.WriteLine("rangle: " + lfa);
            return lfa;
        }

        private double calculateRightArmPam (Skeleton sk)
        {
            double rp;
            vec rap = new vec(1.0, 0.0, 0.0);
            vec arm = getPointVect(sk, JointType.ElbowRight, JointType.ShoulderRight);
            
            if (arm == null)
                return double.NegativeInfinity;
            
            rap.set_y (0.0);
            arm.set_y (0.0);
            
            rp = rap.angle (arm)-(Math.PI)/2;
            //Debug.WriteLine("rangle: " + rp);
            return rp;
        }
        
        private double calculateRightArmLift (Skeleton sk)
        {
            double ral;

            vec re = new vec(0.0, 1.0, 0.0);
            vec arm = getPointVect(sk, JointType.ShoulderRight, JointType.ElbowRight);

            if (arm == null)
                return double.NegativeInfinity;
            
            re.set_x (0.0);
            arm.set_x (0.0);
            
            ral = (Math.PI)/2 - re.angle (arm);
            //Debug.WriteLine("rangle: " + ral);
            return ral;
        }

        private double calculateRightArmRoll(Skeleton sk)
        {
            double rar;
            
            vec re = new vec(1.0, 0.0, 0.0);
            vec arm = getPointVect(sk, JointType.WristRight, JointType.ElbowRight);

            if (arm == null)
                return double.NegativeInfinity;

            re.set_z(0.0);
            arm.set_z(0.0);
            rar = (Math.PI)/2-re.angle(arm);
            
            //Debug.WriteLine("rangle: " + rar);
            return rar;
        }

        private double calculateRightforeArmLift(Skeleton sk)
        {
            double rfa;
            vec rvc = getPointVect(sk, JointType.WristRight, JointType.ElbowRight);
            vec arm = getPointVect(sk, JointType.ElbowRight, JointType.ShoulderRight);

            if (arm == null || rvc == null)
                return double.NegativeInfinity;
            
            rfa = -rvc.angle(arm);
            //Debug.WriteLine("rangle: " + rfa);
            return rfa;
        }
        
        private void dumpPositions (Skeleton skeleton)
        {
            Trace.WriteLine ("------");
            Trace.WriteLine ("Shoulder Center:" + dumpPoint (skeleton, JointType.ShoulderCenter));
            Trace.WriteLine ("---");
            Trace.WriteLine("Shoulder Left:" + dumpPoint(skeleton, JointType.ShoulderLeft));
            Trace.WriteLine("Elbow Left:" + dumpPoint(skeleton, JointType.ElbowLeft));
            Trace.WriteLine("Wrist Left:" + dumpPoint(skeleton, JointType.WristLeft));
            Trace.WriteLine("Hand Left:" + dumpPoint(skeleton, JointType.HandLeft));
            Trace.WriteLine ("---");
            Trace.WriteLine("Shoulder Right:" + dumpPoint(skeleton, JointType.ShoulderRight));
            Trace.WriteLine("Elbow Right:" + dumpPoint(skeleton, JointType.ElbowRight));
            Trace.WriteLine("Shoulder Center:" + dumpPoint(skeleton, JointType.WristRight));
            Trace.WriteLine("Shoulder Center:" + dumpPoint(skeleton, JointType.HandLeft));
            Trace.WriteLine ("------");
        }

        private double CalculatePanAngel (vec reference, vec target)
        {
            return 0.0;
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }
    }
}
