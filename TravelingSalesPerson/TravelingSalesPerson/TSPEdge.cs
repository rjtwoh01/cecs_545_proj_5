using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace TravelingSalesPerson
{
    class TSPEdge
    {

        public double length;
        public Point p1;
        public Point p2;

        public TSPEdge(Point point1, Point point2)
        {
            this.p1 = point1;
            this.p2 = point2;

            this.length = TSP.distance(p1, p2);
        }

        public double DistanceFrom(Point point)
        {
            double distance1 = TSP.distance(this.p1, point);
            double distance2 = TSP.distance(this.p2, point);

            return (distance1 + distance2 - this.length);
        }
    }
}
