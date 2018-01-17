using System;
using System.Collections.Generic;
using System.IO;

namespace Kinect_Anywhere
{
    class ConfigurationManager
    {
        public static string COLOR_KEY = "ColorData";
        public static string BODY_KEY = "BodyData";
        public static string POINT_CLOUD_KEY = "PointCloudData";

        public static string COLOR_PORT_KEY = "ColorPort";
        public static string BODY_PORT_KEY = "BodyPort";
        public static string POINT_CLOUD_PORT_KEY = "PointCloudPort";

        private const string CONFIGURATION_FILE_NAME = "Configuration.txt";
        private const bool defColorData = false;
        private const bool defBodyData = false;
        private const bool defPointCloudData = false;
        private const string delimiter = " = ";

        private readonly string defContent = COLOR_KEY + delimiter + defColorData + Environment.NewLine +
                                             BODY_KEY + delimiter + defBodyData + Environment.NewLine +
                                             POINT_CLOUD_KEY + delimiter + defPointCloudData;

        public IDictionary<string, bool> GetConfiguration()
        {
            IDictionary<string, bool> configuration = new Dictionary<string, bool>();

            // This text is added only once to the file.
            if (File.Exists(CONFIGURATION_FILE_NAME))
            {
                string[] lines = File.ReadAllLines(CONFIGURATION_FILE_NAME);
                foreach (string line in lines)
                {
                    string[] content = line.Split('=');
                    string key = content[0].Trim();
                    bool value = Convert.ToBoolean(content[1].Trim());

                    configuration.Add(key, value);
                }
            }
            else
            {
                configuration.Add(COLOR_KEY, defColorData);
                configuration.Add(BODY_KEY, defBodyData);
                configuration.Add(POINT_CLOUD_KEY, defPointCloudData);
            }

            return configuration;
        }

        public void SaveConfiguration(bool colorData, bool bodyData, bool pointCloudData)
        {
            string content = COLOR_KEY + delimiter + colorData + Environment.NewLine +
                             BODY_KEY + delimiter + bodyData + Environment.NewLine +
                             POINT_CLOUD_KEY + delimiter + pointCloudData;
            // Create a file to write to.
            File.WriteAllText(CONFIGURATION_FILE_NAME, content);
        }
    }
}
