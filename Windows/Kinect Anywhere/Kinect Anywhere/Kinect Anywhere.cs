﻿using Microsoft.Kinect;
using System;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using ZeroMQ;
using System.Collections.Generic;

namespace Kinect_Anywhere
{
    public partial class KinectAnywhereForm : Form
    {
        private KinectSensor kinectSensor;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader multiFrameSourceReader;

        private List<byte> pointCloudData = new List<byte>();
        private List<byte> bodyFrameData = new List<byte>();

        private byte[] colorFrameData;

        /// <summary>
        /// Intermediate storage for the extended depth data received from the camera in the current frame
        /// </summary>
        private IntPtr depthFrameData;
        private IntPtr camerSpacePoints;
        private IntPtr colorSpacePoints;
        private Body[] bodyArray;

        /// <summary>
        /// Intermediate storage for the color data received from the camera in 32bit color
        /// </summary>
        private IntPtr colorPixels;

        private ZContext zmqContext;
        private ZSocket pointCloudPublisher;
        private ZSocket colorFramePublisher;
        private ZSocket bodyFramePublisher;

        /// <summary>
        /// We are accounting transparency in RGB image
        /// </summary>
        private const int COLOR_BYTES_PER_PIXEL = 4;

        private int DEPTH_FRAME_LENGTH;
        private uint DEPTH_FRAME_BYTES;
        private uint CAMERA_SPACE_BYTES;
        private uint COLOR_SPACE_BYTES;
        private uint COLOR_PIXEL_BYTES;
        private int COLOR_FRAME_WIDTH;
        private int COLOR_FRAME_HEIGHT;

        private ConfigurationManager ConfigurationManager;
        private readonly Array ALL_JOINTS = Enum.GetValues(typeof(JointType));

        public KinectAnywhereForm()
        {
            InitializeComponent();
            ConfigurationManager = new ConfigurationManager();

            zmqContext = new ZContext();
            zmqContext.SetOption(ZContextOption.IO_THREADS, 3);
            pointCloudPublisher = new ZSocket(zmqContext, ZSocketType.PUB);
            colorFramePublisher = new ZSocket(zmqContext, ZSocketType.PUB);
            bodyFramePublisher = new ZSocket(zmqContext, ZSocketType.PUB);
        }

        private void OnAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            var isAvailable = e.IsAvailable;
            trayIcon.Icon = (isAvailable) ? Properties.Resources.KinectGreen : Properties.Resources.KinectRed;
            trayIcon.Text = (isAvailable) ? Text + " (Running)" : Text + " (Disconnect)";
        }

        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void OnMultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            BodyFrame bodyFrame = null;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();

                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if (depthFrame == null || colorFrame == null || bodyFrame == null)
                {
                    return;
                }

                // Copy color data (using Bgra format)
                colorFrame.CopyConvertedFrameDataToIntPtr(colorPixels, COLOR_PIXEL_BYTES, ColorImageFormat.Bgra);

                ProcessFrames(ref depthFrame, ref bodyFrame);
            }
            finally
            {
                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }

                if (bodyFrame != null)
                {
                    bodyFrame.Dispose();
                }
            }
        }

        unsafe public void ProcessFrames(ref DepthFrame depthFrame, ref BodyFrame bodyFrame)
        {
            if (PointCloudData.Checked)
            {
                depthFrame.CopyFrameDataToIntPtr(depthFrameData, DEPTH_FRAME_BYTES);
                coordinateMapper.MapDepthFrameToCameraSpaceUsingIntPtr(depthFrameData, DEPTH_FRAME_BYTES, camerSpacePoints, CAMERA_SPACE_BYTES);
                coordinateMapper.MapDepthFrameToColorSpaceUsingIntPtr(depthFrameData, DEPTH_FRAME_BYTES, colorSpacePoints, COLOR_SPACE_BYTES);

                int validPointCount = 0;

                // Remove old points
                pointCloudData.Clear();

                //At this point, we are just reserving 4 bytes for storing 'validPointCount' and we are going to modify it later
                AddArrayToList(ref pointCloudData, new byte[4] { 0, 0, 0, 0 });

                var colorSpacePoint = (ColorSpacePoint*)colorSpacePoints;
                var camerSpacePoint = (CameraSpacePoint*)camerSpacePoints;
                var colorPixel = (byte*)colorPixels;

                for (var index = 0; index < DEPTH_FRAME_LENGTH; index++)
                {
                    int colorX = (int)Math.Floor(colorSpacePoint[index].X);
                    int colorY = (int)Math.Floor(colorSpacePoint[index].Y);

                    // Not every depth colorPixel has a corresponding color colorPixel.
                    // So always check whether colorX, colorY are valid or not
                    if (colorX < 0 || colorX >= COLOR_FRAME_WIDTH || colorY < 0 || colorY >= COLOR_FRAME_HEIGHT)
                    {
                        continue;
                    }

                    // Now colorX, colorY are colorPixel coordinates in colorspace,
                    // so to get the index of the corresponding colorPixel,
                    // we need to multiply colorY by COLOR_FRAME_WIDTH
                    int pixelsBaseIndex = (colorY * COLOR_FRAME_WIDTH + colorX) * COLOR_BYTES_PER_PIXEL;

                    byte blue = colorPixel[pixelsBaseIndex];
                    byte green = colorPixel[pixelsBaseIndex + 1];
                    byte red = colorPixel[pixelsBaseIndex + 2];
                    byte alpha = colorPixel[pixelsBaseIndex + 3];

                    AddArrayToList(ref pointCloudData, BitConverter.GetBytes(camerSpacePoint[index].X)); //add 4 bytes for float x
                    AddArrayToList(ref pointCloudData, BitConverter.GetBytes(camerSpacePoint[index].Y)); //add 4 bytes for float y
                    AddArrayToList(ref pointCloudData, BitConverter.GetBytes(camerSpacePoint[index].Z)); //add 4 bytes for float z

                    uint bgra = (uint)((blue << 24) | (green << 16) | (red << 8) | alpha);
                    AddArrayToList(ref pointCloudData, BitConverter.GetBytes(bgra)); //add 4 bytes for unsigned int

                    //Added 16 bytes in one iteration
                    validPointCount++;
                }

                var validPointBytes = BitConverter.GetBytes(validPointCount);//4 bytes
                UpdateList(validPointBytes, ref pointCloudData);

                pointCloudPublisher.Send(new ZFrame(pointCloudData.ToArray()));
            }

            if (ColorData.Checked)
            {
                Marshal.Copy(colorPixels, colorFrameData, 8, (int)COLOR_PIXEL_BYTES);
                colorFramePublisher.Send(new ZFrame(colorFrameData));
            }

            if (BodyData.Checked)
            {
                // Copy data for Body tracking
                bodyArray = new Body[bodyFrame.BodyCount];
                bodyFrame.GetAndRefreshBodyData(bodyArray);

                // Remove old bodies
                bodyFrameData.Clear();

                //At this point, we are just reserving 4 bytes for storing 'bodyCount' and we are going to modify it later
                AddArrayToList(ref bodyFrameData, new byte[4] { 0, 0, 0, 0 });

                int bodyCount = 0;
                foreach (Body body in bodyArray)
                {
                    if (!body.IsTracked)
                    {
                        continue;
                    }

                    AddArrayToList(ref bodyFrameData, BitConverter.GetBytes(body.TrackingId));//add 8 bytes for ulong TrackingId
                    AddArrayToList(ref bodyFrameData, BitConverter.GetBytes(ALL_JOINTS.Length));//add 4 bytes for int TrackingId

                    foreach (JointType jointType in ALL_JOINTS)
                    {
                        var joint = body.Joints[jointType];
                        AddArrayToList(ref bodyFrameData, BitConverter.GetBytes((int)joint.TrackingState));//add 4 bytes for int TrackingState
                        AddArrayToList(ref bodyFrameData, BitConverter.GetBytes((int)joint.JointType));//add 4 bytes for int JointType
                        AddArrayToList(ref bodyFrameData, BitConverter.GetBytes(joint.Position.X));//add 4 bytes for float X
                        AddArrayToList(ref bodyFrameData, BitConverter.GetBytes(joint.Position.Y));//add 4 bytes for float Y
                        AddArrayToList(ref bodyFrameData, BitConverter.GetBytes(joint.Position.Z));//add 4 bytes for float Z
                    }
                    bodyCount++;
                }

                var bodyCountBytes = BitConverter.GetBytes(bodyCount);//4 bytes
                UpdateList(bodyCountBytes, ref bodyFrameData);

                bodyFramePublisher.Send(new ZFrame(bodyFrameData.ToArray()));
            }
        }

        private void AddArrayToList(ref List<byte> destination, byte[] source)
        {
            for (int i = 0; i < source.Length; i++)
            {
                destination.Add(source[i]);
            }
        }

        private void UpdateList(byte[] source, ref List<byte> destination)
        {
            for (int i = 0; i < source.Length; i++)
            {
                destination[i] = source[i];
            }
        }

        private void CloseToolStripMenuItem_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void SetupPublishers()
        {
            if (ColorData.Checked)
            {
                colorFramePublisher.SetOption(ZSocketOption.CONFLATE, 1);
                colorFramePublisher.Bind("tcp://*:10000");
            }

            if (BodyData.Checked)
            {
                bodyFramePublisher.SetOption(ZSocketOption.CONFLATE, 1);
                bodyFramePublisher.Bind("tcp://*:10001");
            }

            if (PointCloudData.Checked)
            {
                pointCloudPublisher.SetOption(ZSocketOption.CONFLATE, 1);
                pointCloudPublisher.Bind("tcp://*:10002");
            }
        }

        private void StartButton_Click(object sender, EventArgs e)
        {
            if (!ColorData.Checked && !BodyData.Checked && !PointCloudData.Checked)
            {
                MessageBox.Show("Please select alteast one data.", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
            }
            else
            {
                ConfigurationManager.SaveConfiguration(ColorData.Checked, BodyData.Checked, PointCloudData.Checked);

                kinectSensor = KinectSensor.GetDefault();

                if (kinectSensor == null)
                {
                    MessageBox.Show("No Kinect device was detected.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                SetupPublishers();

                kinectSensor.IsAvailableChanged += OnAvailableChanged;

                multiFrameSourceReader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body);
                multiFrameSourceReader.MultiSourceFrameArrived += OnMultiSourceFrameArrived;

                kinectSensor.Open();

                coordinateMapper = kinectSensor.CoordinateMapper;
                COLOR_FRAME_WIDTH = kinectSensor.ColorFrameSource.FrameDescription.Width;
                COLOR_FRAME_HEIGHT = kinectSensor.ColorFrameSource.FrameDescription.Height;

                DEPTH_FRAME_LENGTH = kinectSensor.DepthFrameSource.FrameDescription.Width * kinectSensor.DepthFrameSource.FrameDescription.Height;
                DEPTH_FRAME_BYTES = (uint)(DEPTH_FRAME_LENGTH * sizeof(ushort));
                depthFrameData = Marshal.AllocHGlobal((int)DEPTH_FRAME_BYTES);

                CAMERA_SPACE_BYTES = (uint)(DEPTH_FRAME_LENGTH * Marshal.SizeOf(typeof(CameraSpacePoint)));
                camerSpacePoints = Marshal.AllocHGlobal((int)CAMERA_SPACE_BYTES);

                COLOR_SPACE_BYTES = (uint)(DEPTH_FRAME_LENGTH * Marshal.SizeOf(typeof(ColorSpacePoint)));
                colorSpacePoints = Marshal.AllocHGlobal((int)COLOR_SPACE_BYTES);

                COLOR_PIXEL_BYTES = (uint)(COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT * COLOR_BYTES_PER_PIXEL);
                colorPixels = Marshal.AllocHGlobal((int)COLOR_PIXEL_BYTES);

                colorFrameData = new byte[(2 * sizeof(int)) + COLOR_PIXEL_BYTES];

                var widthBytes = BitConverter.GetBytes(COLOR_FRAME_WIDTH); // 4 bytes
                Array.Copy(widthBytes, colorFrameData, widthBytes.Length);

                var heightBytes = BitConverter.GetBytes(COLOR_FRAME_HEIGHT); // 4 bytes
                Array.Copy(heightBytes, 0, colorFrameData, widthBytes.Length, heightBytes.Length);

                Hide();
            }
        }

        private void KinectAnywhereForm_Load(object sender, EventArgs e)
        {
            var configuration = ConfigurationManager.GetConfiguration();

            ColorData.Checked = configuration[ConfigurationManager.COLOR_KEY];
            ColorData.Text = Convert.ToString(ColorData.Checked);

            BodyData.Checked = configuration[ConfigurationManager.BODY_KEY];
            BodyData.Text = Convert.ToString(BodyData.Checked);

            PointCloudData.Checked = configuration[ConfigurationManager.POINT_CLOUD_KEY];
            PointCloudData.Text = Convert.ToString(PointCloudData.Checked);
        }

        private void ColorData_CheckedChanged(object sender, EventArgs e)
        {
            ColorData.Text = Convert.ToString(ColorData.Checked);
        }

        private void BodyData_CheckedChanged(object sender, EventArgs e)
        {
            BodyData.Text = Convert.ToString(BodyData.Checked);
        }

        private void PointCloudData_CheckedChanged(object sender, EventArgs e)
        {
            PointCloudData.Text = Convert.ToString(PointCloudData.Checked);
        }

        private void KinectAnywhereForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (kinectSensor != null)
            {
                kinectSensor.Close();
            }

            zmqContext.Shutdown();
            DisposeSocket(pointCloudPublisher);
            DisposeSocket(colorFramePublisher);
            DisposeSocket(bodyFramePublisher);

            DisposeIntPtr(depthFrameData);
            DisposeIntPtr(camerSpacePoints);
            DisposeIntPtr(colorSpacePoints);
        }

        private void DisposeSocket(ZSocket socket)
        {
            if (socket != null)
            {
                socket.Close();
                socket.Dispose();
            }

        }
        private void DisposeIntPtr(IntPtr intPtr)
        {
            if (intPtr != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(intPtr);
                intPtr = IntPtr.Zero;
            }
        }
    }
}
