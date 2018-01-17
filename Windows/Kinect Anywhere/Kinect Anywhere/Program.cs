using System;
using System.Windows.Forms;

namespace Kinect_Anywhere
{
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new KinectAnywhereForm());
        }
    }
}
