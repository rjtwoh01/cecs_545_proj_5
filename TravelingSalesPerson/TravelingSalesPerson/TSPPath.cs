using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TravelingSalesPerson
{
    class TSPPath : IComparable<TSPPath>
    {
        private double distance;
        public TimeSpan elapsedTime;
        public List<TSPPoint> points;

        public TSPPath(List<TSPPoint> points, double distance)
        {
            this.points = points;
            this.distance = distance;
        }

        public double Fitness()
        {
            return this.distance;
        }

        public int CompareTo(TSPPath path)
        {
            return this.distance.CompareTo(path.distance);
        }
    }
}
