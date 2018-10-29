using System.Windows;

namespace TravelingSalesPerson
{
    class TSPPoint
    {
        public int matrixIndex;
        public Point point;

        public TSPPoint(Point point, int matrixIndex)
        {
            this.matrixIndex = matrixIndex;
            this.point = point;
        }
    }
}
