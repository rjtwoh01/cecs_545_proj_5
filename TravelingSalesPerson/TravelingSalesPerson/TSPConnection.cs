using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Documents;
using System.Windows.Media;
using System.Windows.Shapes;
using Microsoft.Win32;
using System.Diagnostics;

namespace TravelingSalesPerson
{
    class TSPConnection
    {
        public Point startCity;
        public Point? connection1;
        public Point? connection2;
        public Point? connection3;
        public int citiesVisited = 0;

        public TSPConnection(double x, double y)
        {
            this.startCity = new Point(x, y);
        }

        public TSPConnection()
        {
        }

        public override string ToString()
        {
            string finalString = "";

            finalString += "(" + startCity.X + "," + startCity.Y + ")";
            if (connection1 != null) {
                Point connection = connection1 ?? new Point(0, 0);
                finalString += "\n(" + connection.X + "," + connection.Y + ")";
            }
            if (connection2 != null)
            {
                Point connection = connection2 ?? new Point(0, 0);
                finalString += "\n(" + connection.X + "," + connection.Y + ")";
            }
            if (connection3 != null)
            {
                Point connection = connection3 ?? new Point(0, 0);
                finalString += "\n(" + connection.X + "," + connection.Y + ")";
            }

            return finalString;
        }
    }
}