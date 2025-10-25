using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using Microsoft.Kinect;

namespace KinectViewer
{
    public partial class MainWindow : Window
    {
        private KinectSensor kinectSensor;

        // Frame readers
        private ColorFrameReader colorFrameReader;
        private DepthFrameReader depthFrameReader;
        private InfraredFrameReader infraredFrameReader;
        private BodyFrameReader bodyFrameReader;

        // Bitmaps for rendering
        private WriteableBitmap colorBitmap;
        private WriteableBitmap depthBitmap;
        private WriteableBitmap infraredBitmap;

        // Body tracking
        private Body[] bodies;
        private const double JointThickness = 8;
        private const double BoneThickness = 4;
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        private readonly Brush inferredJointBrush = new SolidColorBrush(Color.FromArgb(255, 255, 255, 0));
        private readonly Pen trackedBonePen = new Pen(new SolidColorBrush(Color.FromArgb(255, 0, 255, 0)), BoneThickness);
        private readonly Pen inferredBonePen = new Pen(new SolidColorBrush(Color.FromArgb(255, 128, 128, 0)), BoneThickness);

        // Coordinate mapper for body overlay
        private CoordinateMapper coordinateMapper;

        public MainWindow()
        {
            InitializeComponent();

            // Get the default Kinect sensor
            kinectSensor = KinectSensor.GetDefault();

            if (kinectSensor != null)
            {
                try
                {
                    // Get coordinate mapper
                    coordinateMapper = kinectSensor.CoordinateMapper;

                    // Initialize readers (but don't open yet)
                    colorFrameReader = kinectSensor.ColorFrameSource.OpenReader();
                    depthFrameReader = kinectSensor.DepthFrameSource.OpenReader();
                    infraredFrameReader = kinectSensor.InfraredFrameSource.OpenReader();
                    bodyFrameReader = kinectSensor.BodyFrameSource.OpenReader();

                    // Open the sensor
                    kinectSensor.Open();

                    statusText.Text = "Kinect connected. Select feeds to enable.";
                }
                catch (Exception ex)
                {
                    statusText.Text = $"Error initializing Kinect: {ex.Message}";
                    MessageBox.Show($"Failed to initialize Kinect: {ex.Message}\n\nStack trace:\n{ex.StackTrace}",
                        "Kinect Initialization Error", MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
            else
            {
                statusText.Text = "No Kinect detected!";
                MessageBox.Show("No Kinect sensor detected. Please connect a Kinect and restart the application.",
                    "Kinect Not Found", MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }

        private void ColorCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (colorCheck.IsChecked == true)
            {
                EnableColorStream();
                // Color and Infrared typically can't run simultaneously
                if (infraredCheck.IsChecked == true)
                {
                    infraredCheck.IsChecked = false;
                }
            }
            else
            {
                DisableColorStream();
            }
        }

        private void DepthCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (depthCheck.IsChecked == true)
            {
                EnableDepthStream();
            }
            else
            {
                DisableDepthStream();
            }
        }

        private void InfraredCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (infraredCheck.IsChecked == true)
            {
                EnableInfraredStream();
                // Color and Infrared typically can't run simultaneously
                if (colorCheck.IsChecked == true)
                {
                    colorCheck.IsChecked = false;
                }
            }
            else
            {
                DisableInfraredStream();
            }
        }

        private void BodyCheck_Changed(object sender, RoutedEventArgs e)
        {
            if (bodyCheck.IsChecked == true)
            {
                EnableBodyTracking();
            }
            else
            {
                DisableBodyTracking();
            }
        }

        private void EnableColorStream()
        {
            if (colorFrameReader != null)
            {
                // Create bitmap
                FrameDescription colorFrameDescription = kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
                colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                colorImage.Source = colorBitmap;
                colorImage.Visibility = Visibility.Visible;

                // Subscribe to frame events
                colorFrameReader.FrameArrived += ColorFrameReader_FrameArrived;
            }
        }

        private void DisableColorStream()
        {
            if (colorFrameReader != null)
            {
                colorFrameReader.FrameArrived -= ColorFrameReader_FrameArrived;
                colorImage.Visibility = Visibility.Collapsed;
                colorImage.Source = null;
                colorBitmap = null;
            }
        }

        private void EnableDepthStream()
        {
            if (depthFrameReader != null)
            {
                // Create bitmap
                FrameDescription depthFrameDescription = kinectSensor.DepthFrameSource.FrameDescription;
                depthBitmap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray16, null);
                depthImage.Source = depthBitmap;
                depthImage.Visibility = Visibility.Visible;

                // Subscribe to frame events
                depthFrameReader.FrameArrived += DepthFrameReader_FrameArrived;
            }
        }

        private void DisableDepthStream()
        {
            if (depthFrameReader != null)
            {
                depthFrameReader.FrameArrived -= DepthFrameReader_FrameArrived;
                depthImage.Visibility = Visibility.Collapsed;
                depthImage.Source = null;
                depthBitmap = null;
            }
        }

        private void EnableInfraredStream()
        {
            if (infraredFrameReader != null)
            {
                // Create bitmap
                FrameDescription infraredFrameDescription = kinectSensor.InfraredFrameSource.FrameDescription;
                infraredBitmap = new WriteableBitmap(infraredFrameDescription.Width, infraredFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray16, null);
                infraredImage.Source = infraredBitmap;
                infraredImage.Visibility = Visibility.Visible;

                // Subscribe to frame events
                infraredFrameReader.FrameArrived += InfraredFrameReader_FrameArrived;
            }
        }

        private void DisableInfraredStream()
        {
            if (infraredFrameReader != null)
            {
                infraredFrameReader.FrameArrived -= InfraredFrameReader_FrameArrived;
                infraredImage.Visibility = Visibility.Collapsed;
                infraredImage.Source = null;
                infraredBitmap = null;
            }
        }

        private void EnableBodyTracking()
        {
            if (bodyFrameReader != null && kinectSensor != null)
            {
                bodies = new Body[kinectSensor.BodyFrameSource.BodyCount];
                skeletonCanvas.Visibility = Visibility.Visible;
                bodyFrameReader.FrameArrived += BodyFrameReader_FrameArrived;
            }
        }

        private void DisableBodyTracking()
        {
            if (bodyFrameReader != null)
            {
                bodyFrameReader.FrameArrived -= BodyFrameReader_FrameArrived;
                if (skeletonCanvas != null)
                {
                    skeletonCanvas.Children.Clear();
                    skeletonCanvas.Visibility = Visibility.Collapsed;
                }
                bodies = null;
            }
        }

        private void ColorFrameReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null && colorBitmap != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        colorBitmap.Lock();

                        colorFrame.CopyConvertedFrameDataToIntPtr(
                            colorBitmap.BackBuffer,
                            (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                            ColorImageFormat.Bgra);

                        colorBitmap.AddDirtyRect(new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight));
                        colorBitmap.Unlock();
                    }
                }
            }
        }

        private void DepthFrameReader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null && depthBitmap != null)
                {
                    FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                    using (KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        depthBitmap.Lock();

                        depthFrame.CopyFrameDataToIntPtr(
                            depthBitmap.BackBuffer,
                            (uint)(depthFrameDescription.Width * depthFrameDescription.Height * 2));

                        depthBitmap.AddDirtyRect(new Int32Rect(0, 0, depthBitmap.PixelWidth, depthBitmap.PixelHeight));
                        depthBitmap.Unlock();
                    }
                }
            }
        }

        private void InfraredFrameReader_FrameArrived(object sender, InfraredFrameArrivedEventArgs e)
        {
            using (InfraredFrame infraredFrame = e.FrameReference.AcquireFrame())
            {
                if (infraredFrame != null && infraredBitmap != null)
                {
                    FrameDescription infraredFrameDescription = infraredFrame.FrameDescription;

                    using (KinectBuffer infraredBuffer = infraredFrame.LockImageBuffer())
                    {
                        infraredBitmap.Lock();

                        infraredFrame.CopyFrameDataToIntPtr(
                            infraredBitmap.BackBuffer,
                            (uint)(infraredFrameDescription.Width * infraredFrameDescription.Height * 2));

                        infraredBitmap.AddDirtyRect(new Int32Rect(0, 0, infraredBitmap.PixelWidth, infraredBitmap.PixelHeight));
                        infraredBitmap.Unlock();
                    }
                }
            }
        }

        private void BodyFrameReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null && bodies != null)
                {
                    bodyFrame.GetAndRefreshBodyData(bodies);

                    skeletonCanvas.Children.Clear();

                    foreach (Body body in bodies)
                    {
                        if (body != null && body.IsTracked)
                        {
                            DrawBody(body);
                        }
                    }
                }
            }
        }

        private void DrawBody(Body body)
        {
            // Draw bones
            DrawBone(body, JointType.Head, JointType.Neck);
            DrawBone(body, JointType.Neck, JointType.SpineShoulder);
            DrawBone(body, JointType.SpineShoulder, JointType.SpineMid);
            DrawBone(body, JointType.SpineMid, JointType.SpineBase);
            DrawBone(body, JointType.SpineShoulder, JointType.ShoulderRight);
            DrawBone(body, JointType.SpineShoulder, JointType.ShoulderLeft);
            DrawBone(body, JointType.SpineBase, JointType.HipRight);
            DrawBone(body, JointType.SpineBase, JointType.HipLeft);

            // Right Arm
            DrawBone(body, JointType.ShoulderRight, JointType.ElbowRight);
            DrawBone(body, JointType.ElbowRight, JointType.WristRight);
            DrawBone(body, JointType.WristRight, JointType.HandRight);
            DrawBone(body, JointType.HandRight, JointType.HandTipRight);
            DrawBone(body, JointType.WristRight, JointType.ThumbRight);

            // Left Arm
            DrawBone(body, JointType.ShoulderLeft, JointType.ElbowLeft);
            DrawBone(body, JointType.ElbowLeft, JointType.WristLeft);
            DrawBone(body, JointType.WristLeft, JointType.HandLeft);
            DrawBone(body, JointType.HandLeft, JointType.HandTipLeft);
            DrawBone(body, JointType.WristLeft, JointType.ThumbLeft);

            // Right Leg
            DrawBone(body, JointType.HipRight, JointType.KneeRight);
            DrawBone(body, JointType.KneeRight, JointType.AnkleRight);
            DrawBone(body, JointType.AnkleRight, JointType.FootRight);

            // Left Leg
            DrawBone(body, JointType.HipLeft, JointType.KneeLeft);
            DrawBone(body, JointType.KneeLeft, JointType.AnkleLeft);
            DrawBone(body, JointType.AnkleLeft, JointType.FootLeft);

            // Draw joints
            foreach (JointType jointType in body.Joints.Keys)
            {
                DrawJoint(body.Joints[jointType]);
            }

            // Draw hand states
            DrawHandState(body, HandState.Open, body.HandLeftState, body.Joints[JointType.HandLeft]);
            DrawHandState(body, HandState.Open, body.HandRightState, body.Joints[JointType.HandRight]);
        }

        private void DrawBone(Body body, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = body.Joints[jointType0];
            Joint joint1 = body.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked || joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // Map camera space points to depth space (512x424)
            DepthSpacePoint depthPoint0 = coordinateMapper.MapCameraPointToDepthSpace(joint0.Position);
            DepthSpacePoint depthPoint1 = coordinateMapper.MapCameraPointToDepthSpace(joint1.Position);

            // Validate coordinates - skip if invalid
            if (float.IsInfinity(depthPoint0.X) || float.IsInfinity(depthPoint0.Y) ||
                float.IsInfinity(depthPoint1.X) || float.IsInfinity(depthPoint1.Y) ||
                float.IsNaN(depthPoint0.X) || float.IsNaN(depthPoint0.Y) ||
                float.IsNaN(depthPoint1.X) || float.IsNaN(depthPoint1.Y))
            {
                return;
            }

            // Scale depth space (512x424) to canvas size (1920x1080)
            Point point0 = new Point(
                depthPoint0.X * 1920.0 / 512.0,
                depthPoint0.Y * 1080.0 / 424.0);
            Point point1 = new Point(
                depthPoint1.X * 1920.0 / 512.0,
                depthPoint1.Y * 1080.0 / 424.0);

            // Choose pen based on tracking state
            Pen drawPen = (joint0.TrackingState == TrackingState.Tracked && joint1.TrackingState == TrackingState.Tracked)
                ? trackedBonePen : inferredBonePen;

            Line line = new Line
            {
                X1 = point0.X,
                Y1 = point0.Y,
                X2 = point1.X,
                Y2 = point1.Y,
                Stroke = drawPen.Brush,
                StrokeThickness = drawPen.Thickness
            };

            skeletonCanvas.Children.Add(line);
        }

        private void DrawJoint(Joint joint)
        {
            if (joint.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // Map camera space point to depth space (512x424)
            DepthSpacePoint depthPoint = coordinateMapper.MapCameraPointToDepthSpace(joint.Position);

            // Validate coordinates - skip if invalid
            if (float.IsInfinity(depthPoint.X) || float.IsInfinity(depthPoint.Y) ||
                float.IsNaN(depthPoint.X) || float.IsNaN(depthPoint.Y))
            {
                return;
            }

            // Scale depth space (512x424) to canvas size (1920x1080)
            Point point = new Point(
                depthPoint.X * 1920.0 / 512.0,
                depthPoint.Y * 1080.0 / 424.0);

            Ellipse ellipse = new Ellipse
            {
                Width = JointThickness,
                Height = JointThickness,
                Fill = joint.TrackingState == TrackingState.Tracked ? trackedJointBrush : inferredJointBrush
            };

            Canvas.SetLeft(ellipse, point.X - JointThickness / 2);
            Canvas.SetTop(ellipse, point.Y - JointThickness / 2);

            skeletonCanvas.Children.Add(ellipse);
        }

        private void DrawHandState(Body body, HandState referenceState, HandState handState, Joint handJoint)
        {
            if (handJoint.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // Map hand position
            DepthSpacePoint depthPoint = coordinateMapper.MapCameraPointToDepthSpace(handJoint.Position);

            // Validate coordinates
            if (float.IsInfinity(depthPoint.X) || float.IsInfinity(depthPoint.Y) ||
                float.IsNaN(depthPoint.X) || float.IsNaN(depthPoint.Y))
            {
                return;
            }

            // Scale depth space to canvas
            Point point = new Point(
                depthPoint.X * 1920.0 / 512.0,
                depthPoint.Y * 1080.0 / 424.0);

            // Determine hand state visual
            Brush handBrush = null;
            double handSize = 30;

            switch (handState)
            {
                case HandState.Open:
                    handBrush = new SolidColorBrush(Color.FromArgb(180, 0, 255, 0)); // Green
                    handSize = 40;
                    break;
                case HandState.Closed:
                    handBrush = new SolidColorBrush(Color.FromArgb(180, 255, 0, 0)); // Red
                    handSize = 30;
                    break;
                case HandState.Lasso:
                    handBrush = new SolidColorBrush(Color.FromArgb(180, 0, 0, 255)); // Blue
                    handSize = 35;
                    break;
                case HandState.Unknown:
                case HandState.NotTracked:
                default:
                    return; // Don't draw anything for unknown/not tracked
            }

            // Draw hand state indicator circle
            Ellipse handIndicator = new Ellipse
            {
                Width = handSize,
                Height = handSize,
                Fill = handBrush,
                Stroke = new SolidColorBrush(Color.FromArgb(255, 255, 255, 255)),
                StrokeThickness = 2
            };

            Canvas.SetLeft(handIndicator, point.X - handSize / 2);
            Canvas.SetTop(handIndicator, point.Y - handSize / 2);

            skeletonCanvas.Children.Add(handIndicator);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            // Clean up
            if (colorFrameReader != null)
            {
                colorFrameReader.Dispose();
                colorFrameReader = null;
            }

            if (depthFrameReader != null)
            {
                depthFrameReader.Dispose();
                depthFrameReader = null;
            }

            if (infraredFrameReader != null)
            {
                infraredFrameReader.Dispose();
                infraredFrameReader = null;
            }

            if (bodyFrameReader != null)
            {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }

            if (kinectSensor != null)
            {
                kinectSensor.Close();
                kinectSensor = null;
            }
        }
    }
}