//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Speech.Recognition;
    using System.Speech.AudioFormat;
    using System.Linq;
    using System;
    using System.Collections.Generic;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 900.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 450.0f;

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
        ///
        /// Speech recognition variable
        /// 
        private SpeechRecognitionEngine speechEngine;
        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;
        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        ///
        /// Some necessary variables
        private List<Point> square = new List<Point>();
        private List<Point> circle = new List<Point>();
        private List<Point> triangle = new List<Point>();
        private List<Point> unknown = new List<Point>();
        private List<Point> myDrawn = new List<Point>();
        private bool isChecked = false;
        private int distance = 0;
        // change the coordinate of origin to center of main windows
        private double originX = RenderWidth / 2;
        private double originY = RenderHeight / 2;
        // default color of brush
        private SolidColorBrush brush = Brushes.Yellow;
        // skeleton tracked ID
        //private int currentTrackingId = 0;
        // DrawingGroup drawingGroup = new DrawingGroup();
        //DrawingImage imageSource = new DrawingImage();
        //bool isMouseDown = false;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
            CreateSquare(100);
            CreateCircle(50);
            CreateTriangle(100);
            Instruction.Text = "1. Raise your left hand up over your shoulder to start drawing by your right hand"
                + System.Environment.NewLine + "2. You can draw a circle, a triangle or a rectangle, when you finish drawing, put your left hand down below your shoulder, it will show you the shape you want draw"
                + System.Environment.NewLine + "3. Now you can change the distance between kinect and you by moving, the size of the shape will change smaller if the distance shorter and bigger if the distance longer"
                + System.Environment.NewLine + "4. You can change the color of the shape by speaking the name of color - green, red, blue or yellow"
                + System.Environment.NewLine + "5. If you want to draw another shape, you should repeat from the step 1 and to close application, simply say Quit";
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
            RecognizerInfo ri = (from recognizer in SpeechRecognitionEngine.InstalledRecognizers()
                                 where "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase)
                                 select recognizer).FirstOrDefault();
            if (ri != null)
            {
                this.speechEngine = new SpeechRecognitionEngine(ri.Id);
                var color = new Choices();
                color.Add(new SemanticResultValue("RED", "Red"));
                color.Add(new SemanticResultValue("BLUE", "Blue"));
                color.Add(new SemanticResultValue("GREEN", "Green"));
                color.Add(new SemanticResultValue("YELLOW", "Yellow"));
                color.Add(new SemanticResultValue("QUIT", "Quit"));
                var gb = new GrammarBuilder();
                gb.Culture = ri.Culture;
                gb.Append(color);
                var g = new Grammar(gb);
                speechEngine.LoadGrammar(g);
                this.speechEngine.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(speechEngine_SpeechRecognized); speechEngine.SetInputToAudioStream(sensor.AudioSource.Start(), new SpeechAudioFormatInfo(EncodingFormat.Pcm, 16000,
                         16, 1, 32000, 2, null));
                speechEngine.RecognizeAsync(RecognizeMode.Multiple);
            }
            //content.Text = "1. put your left hand over your shoulder, then use your right hand to draw a shape";
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
            if (this.speechEngine != null)
            {
                this.speechEngine.SpeechRecognized -=new EventHandler<SpeechRecognizedEventArgs>(speechEngine_SpeechRecognized);
                this.speechEngine.RecognizeAsyncStop();
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
                dc.DrawRectangle(Brushes.LightGray, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));
                
                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                            this.DrawFreeformLine(skel, dc);
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

        private PolyLineSegment freeformline = new PolyLineSegment();
        private void DrawFreeformLine(Skeleton skeleton, DrawingContext drawingContext)
        {
            //Console.WriteLine("run");
           
            Joint handLeftJoint = skeleton.Joints[JointType.HandLeft];
            Joint handRightJoint = skeleton.Joints[JointType.HandRight];
            Point handRightPoint = SkeletonPointToScreen(handRightJoint.Position);
            Point handLeftPoint = SkeletonPointToScreen(handLeftJoint.Position);
            Joint shoulderCenterJoint = skeleton.Joints[JointType.ShoulderCenter];
            Point shoulderCenterPoint = SkeletonPointToScreen(shoulderCenterJoint.Position);
            if (handLeftPoint.Y < shoulderCenterPoint.Y) 
                {
                // reset image box in the left
                isChecked = true;
               
                freeformline.Points.Add(handRightPoint);
                myDrawn.Add(handRightPoint);
                for (int i = 0; i < freeformline.Points.Count - 1; i++)
                {
                    drawingContext.DrawLine(new Pen(Brushes.Yellow, 3),
                        freeformline.Points[i], freeformline.Points[i + 1]);
                }
            }
            else
            {
                if (isChecked)
                {
                    copyList(myDrawn, unknown);
                    Console.WriteLine(myDrawn.Count());
                    Console.WriteLine(unknown.Count());
                }
                NormaliseDrawing();
                string shape = RecogniseShape();
               // Console.WriteLine(shape);
                isChecked = false;
                if (shape != null)
                {
                        drawingContext.DrawRectangle(Brushes.LightGray,
                            new Pen(Brushes.LightGray, 6),
                            new Rect(0, 0, RenderWidth, RenderHeight));
                        this.getDistance(skeleton);
                        switch (shape)
                        {
                            case "triangle":
                                drawTriangle(distance / 15, brush, drawingContext);
                            //drawCircle(distance / 10, Brushes.Red, drawingContext);
                            break;
                            case "square":
                                drawRectangle(distance / 15, brush, drawingContext);
                            //drawCircle(distance / 10, Brushes.Red, drawingContext);
                                break;
                            case "circle":
                                drawCircle(distance / 15, brush, drawingContext);
                                break;
                            default:
                            //drawCircle(distance / 10, brush, drawingContext);
                            break;
                        }

                    //}
                }
                myDrawn.Clear();
                freeformline.Points.Clear();
            }
        }

        /// <summary>
        /// this function copy elements of listA to listB
        /// </summary>
        /// <param name="listA"></param>
        /// <param name="listB"></param>
        /// <returns></returns>
        private void copyList(List<Point> listA, List<Point> listB)
        {
            listB.Clear();
            for(int i=0; i< listA.Count(); i++)
            {
                listB.Add(listA[i]);
            }
        }
        /// <summary>
        /// this function to get distance from user to kinect
        /// </summary>
        /// <param name="skeleton"></param>
        private void getDistance(Skeleton skeleton)
        {
            Joint hipCenterJoint = skeleton.Joints[JointType.HipCenter];
            Point hipCenterPoint = SkeletonPointToScreen(hipCenterJoint.Position);
            int depth = SkeletonDepthToScreen(hipCenterJoint.Position);
            distance= depth;
        }
       
       
        /// <summary>
        /// draw rectangle
        /// </summary>
        /// <param name="size"></param>
        /// <param name="brush"></param>
        /// <param name="dc"></param>
        private void drawRectangle(double size, SolidColorBrush brush, DrawingContext dc)
        {
            Point pt = new Point(originX-size/2, originY-size/2);

            Point pt1 = new Point(originX-size/2, originY+size/2);

            Point pt2 = new Point(originX+size/2, originY-size/2);

            Point pt3 = new Point(originX+size/2, originY+size/2);

            dc.DrawLine(new Pen(brush, 3), pt, pt1);
            dc.DrawLine(new Pen(brush, 3), pt, pt2);
            dc.DrawLine(new Pen(brush, 3), pt1, pt3);
            dc.DrawLine(new Pen(brush, 3), pt2, pt3);
        }

        /// <summary>
        /// draw Triangle
        /// </summary>
        /// <param name="size"></param>
        /// <param name="dc"></param>
        private void drawTriangle(double size, SolidColorBrush brush, DrawingContext dc)
        {
            Point pt1 = new Point(originX+size / 2, originY+size/2);
            Point pt2 = new Point(originX, originY-size/2);
            Point pt3 = new Point(originX-size/2, originY+size/2);
            triangle.Add(pt3);
            dc.DrawLine(new Pen(brush, 3), pt1, pt2);
            dc.DrawLine(new Pen(brush, 3), pt2, pt3);
            dc.DrawLine(new Pen(brush, 3), pt1, pt3);
        }
        /// <summary>
        /// draw circle
        /// </summary>
        /// <param name="radius"></param>
        /// <param name="brush"></param>
        /// <param name="dc"></param>
        private void drawCircle(double radius, SolidColorBrush brush, DrawingContext dc)
        {
           dc.DrawEllipse(Brushes.Transparent,
                    new Pen(brush, 3), new Point(originX,originY), radius, radius
                   );
        }
        /// <summary>
        /// this function to recognize voice and change color of shape
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        void speechEngine_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            const double ConfidenceThreshold = 0.3;
            if (e.Result.Confidence >= ConfidenceThreshold)
            {
                switch (e.Result.Semantics.Value.ToString())
                {
                    case "Red":
                        brush = Brushes.Red;
                        break;
                    case "Blue":
                        brush = Brushes.Blue;
                        break;
                    case "Green":
                        brush = Brushes.Green;
                        break;
                    case "Yellow":
                        brush = Brushes.Yellow;
                        break;
                    case "Quit":
                        Application.Current.Shutdown();
                        break;
                    default:
                        //brush = Brushes.Yellow;
                        break;
                }
            }
        }
        /// <summary>
        /// Create Triangle template, Circle template and Rectangle template
        /// </summary>
        private void CreateTriangle(double side)
        {
            Point pt1 = new Point(side/2, 0);
            triangle.Add(pt1);
            Point pt2 = new Point(0, side);
            triangle.Add(pt2);
            Point pt3 = new Point(side, side);
            triangle.Add(pt3);

            for (int i = 1; i < 10; i++)
            {
                Point pt =
                    new Point(side/2 + (pt1.X + pt2.X) * i * 10 / side,
                    (pt1.Y + pt2.Y) * i * 10 / side);
                triangle.Add(pt);
                pt = new Point(pt3.X * i * 10 / side, pt2.Y);
                triangle.Add(pt);
                pt =
                    new Point((pt1.X + pt2.X) * i * 10 / side,
                    side + (pt1.Y - pt2.Y) * i * 10 / side);
                triangle.Add(pt);
            }
        }

        private void CreateCircle(double radius)
        {
            //double radius = 50;
            for (int i = 0; i < 36; i++)
            {
                Point pt =
                    new Point(radius + radius * Math.Cos(i * 10 * 6.28 / 360),
                    radius + radius * Math.Sin(i * 10 * 6.28 / 360));
                circle.Add(pt);
            }
        }

        private void CreateSquare(double sizeEdge)
        {
            // Points for square
            Point pt = new Point(0, 0);
            square.Add(pt);
            pt = new Point(sizeEdge,sizeEdge );
            square.Add(pt);
            pt = new Point(0, sizeEdge);
            square.Add(pt);
            pt = new Point(sizeEdge, 0);
            square.Add(pt);
            for (int i = 1; i < 10; i++)
            {
                // Left side
                pt = new Point(0, i * 10);
                square.Add(pt);
                // Right side
                pt = new Point(sizeEdge, i * 10);
                square.Add(pt);
                // Top side
                pt = new Point(i * 10, 0);
                square.Add(pt);
                // Bottom side
                pt = new Point(i * 10, sizeEdge);
                square.Add(pt);
            }
        }
        ///
        /// 
        private void NormaliseDrawing()
        {
            double maxX = 0;
            double minX = 10E+10;
            double maxY = 0;
            double minY = 10E+10;

            for (int i = 0; i < unknown.Count; i++)
            {
                if (maxX < unknown[i].X)
                    maxX = unknown[i].X;
                if (minX > unknown[i].X)
                    minX = unknown[i].X;
                if (maxY < unknown[i].Y)
                    maxY = unknown[i].Y;
                if (minY > unknown[i].Y)
                    minY = unknown[i].Y;
            }

            for (int i = 0; i < unknown.Count; i++)
            {
                double x = (unknown[i].X - minX) * 100 / (maxX - minX);
                double y = (unknown[i].Y - minY) * 100 / (maxY - minY);
                unknown[i] = new Point(x, y);
            }
        }

        private string RecogniseShape()
        {
            string recognisedShape = "unknown";
            double ut = GetDifference(unknown, triangle);
            double us = GetDifference(unknown, square);
            double uc = GetDifference(unknown, circle);
            if (ut < us && ut < uc)
                recognisedShape = "triangle";
            if (us < ut && us < uc)
                recognisedShape = "square";
            if (uc < ut && uc < us)
                recognisedShape = "circle";

            return recognisedShape;
        }

        private double GetDifference(List<Point> drawing, List<Point> sample)
        {
            double sum = 0;
            for (int i = 0; i < drawing.Count; i++)
            {
                double min = 10E+10;
                for (int k = 0; k < sample.Count; k++)
                {
                    double distance =
                        (drawing[i].X - sample[k].X) * (drawing[i].X - sample[k].X) +
                        (drawing[i].Y - sample[k].Y) * (drawing[i].Y - sample[k].Y);
                    if (min > distance)
                        min = distance;
                }
                sum += min;
            }
           // Console.WriteLine(sum / drawing.Count);
            return sum / drawing.Count;
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
        /// get depth of point
        /// </summary>
        /// <param name="skelpoint"></param>
        /// <returns></returns>
                 
        private int SkeletonDepthToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return depthPoint.Depth;
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