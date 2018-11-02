using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Diagnostics;
using System.Collections;
using System.Security.Cryptography;
using System.IO;

namespace TravelingSalesPerson
{
    class TSP
    {
        public Point canvasOffset; //Not sure if we need this yet
        public Point maxPoint;
        public Point minPoint;
        public List<Point> points;
        public List<Point> tempFinalList;
        public List<TSPConnection> connectedPoints;
        public List<TSPPoint> tspPoints;
        private RNGCryptoServiceProvider cryptoProvider = new RNGCryptoServiceProvider();

        public double[,] matrix;
        public double shortestDistance;

        public TSP(List<Point> points)
        {
            //Values for UI purposes, creating offsets for the grid
            this.points = new List<Point>();
            this.minPoint = points.First();
            this.maxPoint = points.First();
            this.tempFinalList = new List<Point>();
            tspPoints = new List<TSPPoint>();

            foreach (Point point in points)
            {
                this.points.Add(point);                
            }

            for (int i = 0; i < points.Count; i++)
            {
                TSPPoint tspPoint = new TSPPoint(points[i], i);
                tspPoints.Add(tspPoint);
                Point point = new Point(points[i].X, points[i].Y);

                if (point.X < this.minPoint.X) { this.minPoint.X = point.X; }
                else if (point.X > this.maxPoint.X) { this.maxPoint.X = point.X; }
                if (point.Y < this.minPoint.Y) { this.minPoint.Y = point.Y; }
                else if (point.Y > this.maxPoint.Y) { this.maxPoint.Y = point.Y; }
            }

            this.canvasOffset = new Point(10, 10);

            if (this.minPoint.X > 0) { this.canvasOffset.X -= this.minPoint.X; }
            else { this.canvasOffset.X += this.minPoint.X; }
            if (this.minPoint.Y > 0) { this.canvasOffset.X -= this.minPoint.X; }
            else { this.canvasOffset.X += this.minPoint.X; }

            this.shortestDistance = 0;
            this.GenerateMatrix();

            foreach(TSPPoint point in tspPoints)
            {
                Debug.WriteLine("Point: {0} - ({1}, {2})", point.matrixIndex + 1, point.point.X, point.point.Y);
            }
        }

        private void GenerateMatrix()
        {
            this.matrix = new double[this.points.Count, this.points.Count];
            int count = this.matrix.GetLength(0);

            for (int i = 0; i < count; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    if (j == i)
                    {
                        this.matrix[i, j] = 0;
                    }
                    else if (j > i)
                    {
                        double pathDistance = distance(this.points.ElementAt(i), this.points.ElementAt(j));

                        this.matrix[i, j] = pathDistance;
                        this.matrix[j, i] = pathDistance;
                    }
                }
            }
        }

        //Setup for DFS and BFS
        public TSP(List<TSPConnection> tSPConnections)
        {
            this.connectedPoints = new List<TSPConnection>();

            this.minPoint = tSPConnections.First().startCity;
            this.maxPoint = tSPConnections.First().startCity;
            this.tempFinalList = new List<Point>();

            foreach (TSPConnection point in connectedPoints)
            {
                this.connectedPoints.Add(point);
            }

            for (int i = 0; i < connectedPoints.Count; i++)
            {
                Point point = new Point(connectedPoints[i].startCity.X, connectedPoints[i].startCity.Y);

                if (point.X < this.minPoint.X) { this.minPoint.X = point.X; }
                else if (point.X > this.maxPoint.X) { this.maxPoint.X = point.X; }
                if (point.Y < this.minPoint.Y) { this.minPoint.Y = point.Y; }
                else if (point.Y > this.maxPoint.Y) { this.maxPoint.Y = point.Y; }
            }

            this.canvasOffset = new Point(10, 10);

            if (this.minPoint.X > 0) { this.canvasOffset.X -= this.minPoint.X; }
            else { this.canvasOffset.X += this.minPoint.X; }
            if (this.minPoint.Y > 0) { this.canvasOffset.X -= this.minPoint.X; }
            else { this.canvasOffset.X += this.minPoint.X; }

            this.shortestDistance = 0;
        }

        public static double distance(Point pointOne, Point pointTwo)
        {
            return Math.Sqrt(Math.Pow((pointTwo.X - pointOne.X), 2) + Math.Pow((pointTwo.Y - pointOne.Y), 2));
        }

        public List<Point> BruteForce()
        {
            //This final list will represent the correct order - or path - to take
            List<Point> finalList = new List<Point>();
            var tempList = new List<Point>();
            var newList = new List<Point>();
            double localDistance = 0;
            shortestDistance = 0;
            int totalPermutations = 0;
            int initialCount = 0;

            foreach (Point point in this.points)
            {
                tempList.Add(point);
            }

            initialCount = tempList.Count();

            Point firstElement = tempList.First();
            List<Point> rest = tempList;
            rest.RemoveAt(0);

            //Iterate through each permutaion
            foreach (var perm in Permutate(rest, rest.Count()))
            {
                double shortestSoFar = shortestDistance;
                localDistance = 0;
                newList.Clear();
                newList.Add(firstElement); //we start with the same city every time
                //Iterate through each element in this particular permutation
                foreach (var i in perm)
                {
                    //We need to read the element as a string because it is no longer recognized as a point
                    //Once we have the strong, it can be converted back to a point and added to the new list
                    string[] parts = i.ToString().Split(',');
                    Point tempPoint = new Point(Convert.ToDouble(parts[0]), Convert.ToDouble(parts[1]));
                    newList.Add(tempPoint);
                }
                newList.Add(firstElement); //we end with the same city every time
                //Calculate the distance
                for (int i = 0; i < newList.Count(); i++)
                {
                    if ((i + 1) != newList.Count())
                        localDistance += distance(newList[i], newList[i + 1]);
                }
                //Check if this should be a canidate for the final list
                if (shortestDistance > localDistance || shortestDistance == 0)
                {
                    shortestDistance = localDistance;
                    finalList.Clear();
                    finalList = newList.ToList(); //Save computation time of foreach
                }
            }

            int city = 1;
            Debug.WriteLine("\nFinal list: ");
            foreach (Point point in finalList)
            {
                Debug.WriteLine(city + ": " + point);
                city++;
            }
            Debug.WriteLine("\nTotal Run Distance: " + shortestDistance + "\nTotal Permutations: " + totalPermutations);

            return finalList;
        }

        public List<Point> BFS()
        {
            List<Point> finalList = new List<Point>();

            return finalList;
        }

        public List<Point> DFS(List<TSPConnection> tspConnections)
        {
            List<Point> finalList = new List<Point>();
            List<TSPConnection> currentPath = new List<TSPConnection>();
            TSPConnection currentCity = new TSPConnection();
            double localDistance = 0;
            int connection = 0;
            shortestDistance = 0; //reset shortest distance just in case another algorithm was used and shortest distance still has that value
            tempFinalList.Clear();

            currentCity = tspConnections.First();
            Point finalCity = tspConnections.Last().startCity;
            tempFinalList.Add(currentCity.startCity);

            while (tspConnections.First().citiesVisited != 4)
            {

                currentCity = tspConnections.Find(x => x.startCity == tspConnections.ElementAt(connection).startCity);
                currentCity.citiesVisited++;
                if (currentCity.citiesVisited == 1)
                {
                    //Check if the first connection is the goal city
                    if (currentCity.connection1 == finalCity)
                    {
                        tempFinalList.Add(finalCity);
                        localDistance += distance(currentCity.startCity, finalCity);
                        //If this distance of the list is now less than the previous shortest distance
                        //we need to copy this list over to the final list                    
                        if (shortestDistance > localDistance || shortestDistance == 0)
                        {
                            finalList.Clear();
                            finalList.AddRange(tempFinalList);
                            shortestDistance = localDistance;
                        }

                        tempFinalList.Remove(finalCity);
                        localDistance -= distance(currentCity.startCity, finalCity);
                        Point tempFinal = tempFinalList.Last();
                        connection = tspConnections.FindIndex(x => x.startCity == tempFinal);
                    }
                    else
                    {
                        Point connection1 = currentCity.connection1 ?? new Point(0, 0);
                        tempFinalList.Add(connection1);
                        localDistance += distance(currentCity.startCity, connection1);
                        connection = tspConnections.FindIndex(x => x.startCity == connection1);
                    }
                }
                else if (currentCity.citiesVisited == 2 && currentCity.connection2 != null)
                {
                    if (currentCity.connection2 == finalCity)
                    {
                        tempFinalList.Add(finalCity);
                        localDistance += distance(currentCity.startCity, finalCity);
                        if (shortestDistance > localDistance || shortestDistance == 0)
                        {
                            finalList.Clear();
                            finalList.AddRange(tempFinalList);
                            shortestDistance = localDistance;
                        }

                        tempFinalList.Remove(finalCity);
                        localDistance -= distance(currentCity.startCity, finalCity);
                        Point tempFinal = tempFinalList.Last();
                        connection = tspConnections.FindIndex(x => x.startCity == tempFinal);
                    }
                    else
                    {
                        Point connection2 = currentCity.connection2 ?? new Point(0, 0);
                        tempFinalList.Add(connection2);
                        localDistance += distance(currentCity.startCity, connection2);
                        connection = tspConnections.FindIndex(x => x.startCity == connection2);
                    }
                }
                else if (currentCity.citiesVisited == 3 && currentCity.connection3 != null)
                {
                    if (currentCity.connection3 == finalCity)
                    {
                        tempFinalList.Add(finalCity);
                        localDistance += distance(currentCity.startCity, finalCity);
                        if (shortestDistance > localDistance || shortestDistance == 0)
                        {
                            finalList.Clear();
                            finalList.AddRange(tempFinalList);
                            shortestDistance = localDistance;
                        }

                        tempFinalList.Remove(finalCity);
                        localDistance -= distance(currentCity.startCity, finalCity);
                        Point tempFinal = tempFinalList.Last();
                        connection = tspConnections.FindIndex(x => x.startCity == tempFinal);
                    }
                    else
                    {
                        Point connection3 = currentCity.connection3 ?? new Point(0, 0);
                        tempFinalList.Add(connection3);
                        localDistance += distance(currentCity.startCity, connection3);
                        connection = tspConnections.FindIndex(x => x.startCity == connection3);
                    }
                }
                //We have exhausted all possible connections, time to remove and try again
                else if (currentCity.citiesVisited == 4 && connection != 0)
                {
                    Point connectionToRemove = tspConnections.ElementAt(connection).startCity;
                    tempFinalList.Remove(connectionToRemove);
                    currentCity.citiesVisited = 0;
                    localDistance -= distance(tempFinalList.Last(), connectionToRemove);
                    Point tempFinal = tempFinalList.Last();
                    connection = tspConnections.FindIndex(x => x.startCity == tempFinal);
                }

            }

            return finalList;
        }

        public List<Point> BFS(List<TSPConnection> tspConnections)
        {
            List<Point> finalList = new List<Point>();
            List<TSPConnection> currentPath = new List<TSPConnection>();
            TSPConnection currentCity = new TSPConnection();
            List<Point> usedCities = new List<Point>();
            List<List<Point>> combinations = new List<List<Point>>();
            List<List<Point>> tempCombinations = new List<List<Point>>();
            bool found = false;
            double localDistance = 0;
            int connection = 0;
            shortestDistance = 0; //reset shortest distance just in case another algorithm was used and shortest distance still has that value
            tempFinalList.Clear();

            currentCity = tspConnections.First();
            Point finalCity = tspConnections.Last().startCity;
            tempFinalList.Add(currentCity.startCity);
            combinations.Add(copyLists(tempFinalList));
            usedCities.Add(currentCity.startCity);

            while (!found)
            {
                //Each possible combination needs to be iterated through
                foreach (List<Point> var in combinations)
                {
                    tempFinalList.Clear();
                    //we start at the top and see if we can find the fastest route to the final city
                    currentCity = tspConnections.Find(x => x.startCity == var.Last());
                    Point connection1 = currentCity.connection1 ?? new Point(0, 0);
                    Point connection2 = currentCity.connection2 ?? new Point(0, 0);
                    Point connection3 = currentCity.connection3 ?? new Point(0, 0);
                    if ((currentCity.connection1 != null && currentCity.connection2 != null && currentCity.connection3 != null) && (connection1 == finalCity || connection2 == finalCity || connection3 == finalCity))
                    {
                        found = true; //we are done
                    }
                    tempFinalList.AddRange(var);
                    if(!usedCities.Contains(connection1))
                    {
                        tempFinalList.Add(connection1);
                        usedCities.Add(connection1);
                        tempCombinations.Add(copyLists(tempFinalList));
                        tempFinalList.Clear();
                        tempFinalList.AddRange(var);
                    }
                    if (currentCity.connection2 != null && !usedCities.Contains(connection2))
                    {
                        tempFinalList.Add(connection2);
                        usedCities.Add(connection2);
                        tempCombinations.Add(copyLists(tempFinalList));
                        tempFinalList.Clear();
                        tempFinalList.AddRange(var);
                    }
                    if (currentCity.connection3 != null && !usedCities.Contains(connection3))
                    {
                        tempFinalList.Add(connection3);
                        usedCities.Add(connection3);
                        tempCombinations.Add(copyLists(tempFinalList));
                        tempFinalList.Clear();
                        tempFinalList.AddRange(var);
                    }
                }
                combinations.Clear();
                combinations.AddRange(tempCombinations);
                tempCombinations.Clear();
            }

            tempFinalList.Clear();
            //the final list is in combinations.first
            tempFinalList.AddRange(combinations.First());
            for (int i = 0; i < tempFinalList.Count - 1; i++)
            {
                localDistance += distance(tempFinalList[i], tempFinalList[i + 1]);
            }
            shortestDistance = localDistance;
            foreach (Point p in tempFinalList)
            {
                finalList.Add(p);
            }

            return finalList;
        }

        public List<Point> ClosestEdgeInsertion()
        {
            //This final list will represent the correct order - or path - to take
            List<Point> finalList = new List<Point>();
            List<TSPEdge> edges = new List<TSPEdge>();
            //var tempList = new List<Point>();
            //var newList = new List<Point>();
            double localDistance = 0;
            shortestDistance = 0;
            int initialCount = 0;
            
            for (int i = 0; i < this.points.Count; i++)
            {
                //Initially we're going to start with the first 3 points
                if (i < 3)
                {
                    int j = i + 1;
                    if (j == 3) { j = 0; }
                    edges.Add(new TSPEdge(this.points[i], this.points[j]));
                    Debug.WriteLine(i + ", " + j);
                }
                else
                {
                    TSPEdge current = edges[0];
                    localDistance = edges[0].DistanceFrom(this.points[i]);

                    for (int j = 1; j < edges.Count; j++)
                    {
                        double tempDistance = edges[j].DistanceFrom(this.points[i]);
                        if (tempDistance < localDistance)
                        {
                            localDistance = tempDistance;
                            current = edges[j];
                        }
                    }
                    int index = edges.IndexOf(current);
                    edges.Insert(index, new TSPEdge(current.p1, this.points[i]));
                    edges.Insert(index + 1, new TSPEdge(this.points[i], current.p2));
                    edges.Remove(current);
                }
            }

            foreach (TSPEdge edge in edges)
            {
                finalList.Add(edge.p1);
            }
            finalList.Add(edges[0].p1);

            localDistance = 0;
            for (int i = 0; i < finalList.Count(); i++)
            {
                if ((i + 1) != finalList.Count())
                    localDistance += distance(finalList[i], finalList[i + 1]);
            }
            this.shortestDistance = localDistance;

            return finalList;
        }

        public List<Point> copyLists(List<Point> temp)
        {
            List<Point> temp2 = new List<Point>();
            temp2.AddRange(temp);
            return temp2;
        }

        #region Genetic Algorithm

        private void DetermineEndPoints(int index, SortedList<int, TSPConnectionMap> filledConnections, out int endPoint1, out int endPoint2)
        {
            endPoint1 = -1;
            endPoint2 = -1;

            int previous = index;
            int current = filledConnections[index].parent1Connection1;

            endPointsHelper(ref previous, ref current, filledConnections, index);

            if (previous != index)
            {
                endPoint1 = previous;
            }

            previous = index;
            current = filledConnections[index].parent1Connection2;

            endPointsHelper(ref previous, ref current, filledConnections, index);

            if (previous != index)
            {
                endPoint2 = previous;
            }

        }

        private void endPointsHelper(ref int previous, ref int current, SortedList<int, TSPConnectionMap> filledConnections, int index)
        {
            while (true)
            {
                if (current == -1 || current == index)
                {
                    break;
                }

                int tempCurrent;
                if (filledConnections[current].parent1Connection1 != previous)
                {
                    tempCurrent = filledConnections[current].parent1Connection1;
                }
                else
                {
                    tempCurrent = filledConnections[current].parent1Connection2;
                }

                previous = current;
                current = tempCurrent;
            }
        }

        private double FirstHalfDistance(TSPPath chromosome)
        {
            double distance = 0;

            for (int i = 1; i < Math.Floor((double)chromosome.points.Count / (double)2); i++)
            {
                distance += this.matrix[chromosome.points[i - 1].matrixIndex, chromosome.points[i].matrixIndex];
            }

            return distance;
        }

        //Taken from: https://stackoverflow.com/questions/273313/randomize-a-listt
        private List<TSPPoint> FisherYatesShuffle()
        {
            List<TSPPoint> shuffledList = this.tspPoints.ToList();

            int n = shuffledList.Count;

            while (n > 1)
            {
                var box = new byte[1];
                do this.cryptoProvider.GetBytes(box);
                while (!(box[0] < n * (Byte.MaxValue / n)));
                var k = (box[0] % n);
                n--;
                var value = shuffledList[k];
                shuffledList[k] = shuffledList[n];
                shuffledList[n] = value;
            }

            return shuffledList;
        }

        private List<T> genericFisherYatesShuff<T>(List<T> list)
        {
            int n = list.Count;

            while (n > 1)
            {
                var box = new byte[1];
                do this.cryptoProvider.GetBytes(box);
                while (!(box[0] < n * (Byte.MaxValue / n)));
                var k = (box[0] % n);
                n--;
                var value = list[k];
                list[k] = list[n];
                list[n] = value;
            }

            return list;
        }

        private Metrics GenerateMetrics(List<TSPPath> population)
        {
            Metrics metrics = new Metrics();

            metrics.min = population[0].Fitness();
            metrics.max = population[population.Count - 1].Fitness();

            double sum = 0;
            foreach (TSPPath chromosome in population)
            {
                sum += chromosome.Fitness();
            }

            metrics.average = sum / Convert.ToDouble(population.Count);

            sum = 0;
            foreach (TSPPath chromosome in population)
            {
                sum += Math.Pow(chromosome.Fitness() - metrics.average, 2);
            }

            metrics.standardDeviation = Math.Sqrt(sum / Convert.ToDouble(population.Count));

            return metrics;
        }

        private TSPPath GenerateOffspring(TSPPath parent1, TSPPath parent2, int mutationProbability)
        {
            int startParent = GetRandomInt(0, 1);
            int crossoverIndex = GetRandomInt(0, parent1.points.Count - 1);

            int shift = GetRandomInt(0, parent1.points.Count - 2);

            List<TSPPoint> offspringPoints = new List<TSPPoint>();

            //If first half of parent1 is shorter distance than first half of parent2
            if (startParent == 0)
            {
                List<TSPPoint> movedPoints = new List<TSPPoint>();

                for (int i = 0; i < shift; i++)
                {
                    movedPoints.Add(parent1.points[i]);
                }

                parent1.points.RemoveRange(0, shift);
                parent1.points.InsertRange(parent1.points.Count - 1, movedPoints);

                //Add points for first half of parent1
                for (int i = 0; i < crossoverIndex; i++)
                {
                    offspringPoints.Add(parent1.points[i]);
                }

                //Add remaining points from parent2
                for (int i = 0; i < parent2.points.Count; i++)
                {
                    if (!offspringPoints.Contains(parent2.points[i]))
                    {
                        offspringPoints.Add(parent2.points[i]);
                    }
                }
            }
            //If first half of parent2 is shorter than first half of parent1
            else
            {
                for (int i = 0; i < shift; i++)
                {
                    TSPPoint firstPoint = parent1.points[0];
                    parent1.points.RemoveAt(0);
                    parent1.points.Add(firstPoint);
                }

                //add points for first half of parent2
                for (int i = 0; i < crossoverIndex; i++)
                {
                    offspringPoints.Add(parent2.points[i]);
                }

                //Add remaining points from parent1
                for (int i = 0; i < parent1.points.Count; i++)
                {
                    if (!offspringPoints.Contains(parent1.points[i]))
                    {
                        offspringPoints.Add(parent1.points[i]);
                    }
                }
            }

            MutateOffspring(offspringPoints, mutationProbability);

            return new TSPPath(offspringPoints, CalculateDistance(offspringPoints, true));
        }

        #region WOCOffSpring

        private TSPPath GenerateOffSpringWOC(TSPPath parent1, TSPPath parent2)
        {
            if (parent1.points.Count <= 1)
            {
                return new TSPPath(parent1.points.ToList(), CalculateDistance(parent1.points, true));
            }

            List<TSPPoint> offspring = new List<TSPPoint>();
            SortedList<int, TSPConnectionMap> connections = new SortedList<int, TSPConnectionMap>();
            SortedList<int, TSPConnectionMap> filledConnections = new SortedList<int, TSPConnectionMap>();
            SortedList<int, int> missingConnections = new SortedList<int, int>();
            List<TSPPoint> offspringPoints = new List<TSPPoint>();

            MapConnections(ref parent1, ref parent2, ref connections);
            InitializeOffSpring(ref connections, ref filledConnections);
            AddSharedConnections(ref connections, ref filledConnections);
            DetermineMissingConnections(ref filledConnections, ref missingConnections);
            AddConnectionsParent(ref connections, ref filledConnections, ref missingConnections);
            AddRemainingConnections(ref filledConnections, ref missingConnections);
            GenerateOffSpringList(ref parent1, ref parent2, ref filledConnections, ref offspringPoints);

            return new TSPPath(offspringPoints.ToList(), CalculateDistance(offspringPoints, true));
        }

        private void MapConnections(ref TSPPath parent1, ref TSPPath parent2, ref SortedList<int, TSPConnectionMap> connections)
        {
            for (int i = 2; i < parent1.points.Count; i++)
            {
                TSPConnectionMap connection = new TSPConnectionMap();
                connection.parent1Connection1 = parent1.points[i - 2].matrixIndex;
                connection.parent1Connection2 = parent1.points[i].matrixIndex;
                connection.parent1Index = i - 1;
                connections.Add(parent1.points[i - 1].matrixIndex, connection);
            }

            TSPConnectionMap parent1FirstConnection = new TSPConnectionMap();
            parent1FirstConnection.parent1Connection1 = parent1.points.Last().matrixIndex;
            parent1FirstConnection.parent1Connection2 = parent1.points[1].matrixIndex;
            parent1FirstConnection.parent1Index = 0;
            connections.Add(parent1.points[0].matrixIndex, parent1FirstConnection);

            TSPConnectionMap parent1LastConnection = new TSPConnectionMap();
            parent1LastConnection.parent1Connection1 = parent1.points[parent1.points.Count - 2].matrixIndex;
            parent1LastConnection.parent1Connection2 = parent1.points[0].matrixIndex;
            parent1LastConnection.parent1Index = parent1.points.Count - 1;
            connections.Add(parent1.points[parent1.points.Count - 1].matrixIndex, parent1LastConnection);

            for (int i = 2; i < parent2.points.Count; i++)
            {
                TSPConnectionMap connection = connections[parent2.points[i - 1].matrixIndex];
                connection.parent2Connection1 = parent2.points[i - 2].matrixIndex;
                connection.parent2Connection2 = parent2.points[i].matrixIndex;
            }

            TSPConnectionMap parent2FirstConnection = connections[parent2.points[0].matrixIndex];
            parent2FirstConnection.parent2Connection1 = parent2.points.Last().matrixIndex;
            parent2FirstConnection.parent2Connection2 = parent2.points[1].matrixIndex;

            TSPConnectionMap parent2LastConnection = connections[parent2.points[parent2.points.Count - 1].matrixIndex];
            parent2LastConnection.parent2Connection1 = parent2.points[parent2.points.Count - 2].matrixIndex;
            parent2LastConnection.parent2Connection2 = parent2.points[0].matrixIndex;
        }

        private void InitializeOffSpring(ref SortedList<int, TSPConnectionMap> connections, ref SortedList<int, TSPConnectionMap> filledConnections)
        {            
            for (int i = 0; i < connections.Count; i++)
            {
                TSPConnectionMap connection = new TSPConnectionMap();
                connection.parent1Index = connections[i].parent1Index;
                connection.parent1Connection1 = -1;
                connection.parent1Connection2 = -1;
                filledConnections.Add(i, connection);
            }
        }

        private void AddSharedConnections(ref SortedList<int, TSPConnectionMap> connections, ref SortedList<int, TSPConnectionMap> filledConnections)
        {
            // Add connections that exist in both solutions
            for (int i = 0; i < connections.Count; i++)
            {
                // If parent1 connection1 is same as one of parent2's connections
                if (connections[i].parent1Connection1 == connections[i].parent2Connection1
                    || connections[i].parent1Connection1 == connections[i].parent2Connection2)
                {
                    // If connection already exists
                    if (filledConnections[i].parent1Connection1 == connections[i].parent1Connection1
                        || filledConnections[i].parent1Connection2 == connections[i].parent1Connection1)
                    {
                        continue;
                    }

                    // If parent1 connection1 is still available in solution
                    if (filledConnections[i].parent1Connection1 == -1)
                    {
                        // Get connections for shared connection
                        int cConn1Index = filledConnections[connections[i].parent1Connection1].parent1Connection1;
                        int cConn2Index = filledConnections[connections[i].parent1Connection1].parent1Connection2;

                        if (cConn1Index == -1)
                        {
                            filledConnections[i].parent1Connection1 = connections[i].parent1Connection1;
                            filledConnections[connections[i].parent1Connection1].parent1Connection1 = i;
                        }
                        else if (cConn2Index == -1)
                        {
                            filledConnections[i].parent1Connection1 = connections[i].parent1Connection1;
                            filledConnections[connections[i].parent1Connection1].parent1Connection2 = i;
                        }
                    }
                    // If parent1 connection2 is still available in solution
                    else if (filledConnections[i].parent1Connection2 == -1)
                    {
                        // Get connections for shared connection
                        int cConn1Index = filledConnections[connections[i].parent1Connection1].parent1Connection1;
                        int cConn2Index = filledConnections[connections[i].parent1Connection1].parent1Connection2;

                        if (cConn1Index == -1)
                        {
                            filledConnections[i].parent1Connection2 = connections[i].parent1Connection1;
                            filledConnections[connections[i].parent1Connection1].parent1Connection1 = i;
                        }
                        else if (cConn2Index == -1)
                        {
                            filledConnections[i].parent1Connection2 = connections[i].parent1Connection1;
                            filledConnections[connections[i].parent1Connection1].parent1Connection2 = i;
                        }
                    }
                }

                // If parent1 connection2 is same as one of parent2's connections
                if (connections[i].parent1Connection2 == connections[i].parent2Connection1
                    || connections[i].parent1Connection2 == connections[i].parent2Connection2)
                {
                    // If connection already exists
                    if (filledConnections[i].parent1Connection1 == connections[i].parent1Connection2
                        || filledConnections[i].parent1Connection2 == connections[i].parent1Connection2)
                    {
                        continue;
                    }

                    // If parent1 connection1 is still availabile in solution
                    if (filledConnections[i].parent1Connection1 == -1)
                    {
                        // Get connections for shared connection
                        int cConn1Index = filledConnections[connections[i].parent1Connection2].parent1Connection1;
                        int cConn2Index = filledConnections[connections[i].parent1Connection2].parent1Connection2;

                        if (cConn1Index == -1)
                        {
                            filledConnections[i].parent1Connection1 = connections[i].parent1Connection2;
                            filledConnections[connections[i].parent1Connection2].parent1Connection1 = i;
                        }
                        else if (cConn2Index == -1)
                        {
                            filledConnections[i].parent1Connection1 = connections[i].parent1Connection2;
                            filledConnections[connections[i].parent1Connection2].parent1Connection2 = i;
                        }
                    }
                    // If parent1 connection2 is still available in solution
                    else if (filledConnections[i].parent1Connection2 == -1)
                    {
                        // Get connections for shared connection
                        int cConn1Index = filledConnections[connections[i].parent1Connection2].parent1Connection1;
                        int cConn2Index = filledConnections[connections[i].parent1Connection2].parent1Connection2;

                        if (cConn1Index == -1)
                        {
                            filledConnections[i].parent1Connection2 = connections[i].parent1Connection2;
                            filledConnections[connections[i].parent1Connection2].parent1Connection1 = i;
                        }
                        else if (cConn2Index == -1)
                        {
                            filledConnections[i].parent1Connection2 = connections[i].parent1Connection2;
                            filledConnections[connections[i].parent1Connection2].parent1Connection2 = i;
                        }
                    }
                }
            }
        }

        private void DetermineMissingConnections(ref SortedList<int, TSPConnectionMap> filledConnections, ref SortedList<int, int> missingConnections)
        {
            // Determine missing connections
            for (int i = 0; i < filledConnections.Count; i++)
            {
                // If point is missing both connections
                if (filledConnections[i].parent1Connection1 == -1 && filledConnections[i].parent1Connection2 == -1)
                {
                    missingConnections.Add(i, 2);
                }
                // If point is missing one connection
                else if (filledConnections[i].parent1Connection1 == -1 || filledConnections[i].parent1Connection2 == -1)
                {
                    missingConnections.Add(i, 1);
                }
            }
        }

        private void AddConnectionsParent(ref SortedList<int, TSPConnectionMap> connections, ref SortedList<int, TSPConnectionMap> filledConnections, ref SortedList<int, int> missingConnections)
        {
            int count = missingConnections.Count;
            for (int i = 0; i < count; i++)
            {
                if (i >= missingConnections.Count)
                {
                    break;
                }

                int missingConnection = missingConnections.ElementAt(i).Key;

                int endPoint1;
                int endPoint2;

                DetermineEndPoints(missingConnection, filledConnections, out endPoint1, out endPoint2);

                int random = GetRandomInt(0, 1);

                if (random == 0)
                {
                    if (missingConnections.ContainsKey(connections[missingConnection].parent1Connection1)
                        && endPoint1 != connections[missingConnection].parent1Connection1
                        && endPoint2 != connections[missingConnection].parent1Connection1)
                    {
                        if (filledConnections[missingConnection].parent1Connection1 == -1)
                        {
                            filledConnections[missingConnection].parent1Connection1 = connections[missingConnection].parent1Connection1;
                        }
                        else
                        {
                            filledConnections[missingConnection].parent1Connection2 = connections[missingConnection].parent1Connection1;
                        }

                        if (missingConnections[missingConnection] == 1)
                        {
                            missingConnections.Remove(missingConnection);
                        }
                        else
                        {
                            missingConnections[missingConnection] -= 1;
                        }

                        if (filledConnections[connections[missingConnection].parent1Connection1].parent1Connection1 == -1)
                        {
                            filledConnections[connections[missingConnection].parent1Connection1].parent1Connection1 = missingConnection;
                        }
                        else
                        {
                            filledConnections[connections[missingConnection].parent1Connection1].parent1Connection2 = missingConnection;
                        }

                        if (missingConnections[connections[missingConnection].parent1Connection1] == 1)
                        {
                            missingConnections.Remove(connections[missingConnection].parent1Connection1);
                        }
                        else
                        {
                            missingConnections[connections[missingConnection].parent1Connection1] -= 1;
                        }
                    }
                    else if (missingConnections.ContainsKey(connections[missingConnection].parent1Connection2)
                        && endPoint1 != connections[missingConnection].parent1Connection2
                        && endPoint2 != connections[missingConnection].parent1Connection2)
                    {
                        if (filledConnections[missingConnection].parent1Connection1 == -1)
                        {
                            filledConnections[missingConnection].parent1Connection1 = connections[missingConnection].parent1Connection2;
                        }
                        else
                        {
                            filledConnections[missingConnection].parent1Connection2 = connections[missingConnection].parent1Connection2;
                        }

                        if (missingConnections[missingConnection] == 1)
                        {
                            missingConnections.Remove(missingConnection);
                        }
                        else
                        {
                            missingConnections[missingConnection] -= 1;
                        }

                        if (filledConnections[connections[missingConnection].parent1Connection2].parent1Connection1 == -1)
                        {
                            filledConnections[connections[missingConnection].parent1Connection2].parent1Connection1 = missingConnection;
                        }
                        else
                        {
                            filledConnections[connections[missingConnection].parent1Connection2].parent1Connection2 = missingConnection;
                        }

                        if (missingConnections[connections[missingConnection].parent1Connection2] == 1)
                        {
                            missingConnections.Remove(connections[missingConnection].parent1Connection2);
                        }
                        else
                        {
                            missingConnections[connections[missingConnection].parent1Connection2] -= 1;
                        }
                    }
                }
                else
                {
                    if (missingConnections.ContainsKey(connections[missingConnection].parent2Connection1)
                        && endPoint1 != connections[missingConnection].parent2Connection1
                        && endPoint2 != connections[missingConnection].parent2Connection1)
                    {
                        if (filledConnections[missingConnection].parent1Connection1 == -1)
                        {
                            filledConnections[missingConnection].parent1Connection1 = connections[missingConnection].parent2Connection1;
                        }
                        else
                        {
                            filledConnections[missingConnection].parent1Connection2 = connections[missingConnection].parent2Connection1;
                        }

                        if (missingConnections[missingConnection] == 1)
                        {
                            missingConnections.Remove(missingConnection);
                        }
                        else
                        {
                            missingConnections[missingConnection] -= 1;
                        }

                        if (filledConnections[connections[missingConnection].parent2Connection1].parent1Connection1 == -1)
                        {
                            filledConnections[connections[missingConnection].parent2Connection1].parent1Connection1 = missingConnection;
                        }
                        else
                        {
                            filledConnections[connections[missingConnection].parent2Connection1].parent1Connection2 = missingConnection;
                        }

                        if (missingConnections[connections[missingConnection].parent2Connection1] == 1)
                        {
                            missingConnections.Remove(connections[missingConnection].parent2Connection1);
                        }
                        else
                        {
                            missingConnections[connections[missingConnection].parent2Connection1] -= 1;
                        }
                    }
                    else if (missingConnections.ContainsKey(connections[missingConnection].parent2Connection2)
                        && endPoint1 != connections[missingConnection].parent2Connection2
                        && endPoint2 != connections[missingConnection].parent2Connection2)
                    {
                        if (filledConnections[missingConnection].parent1Connection1 == -1)
                        {
                            filledConnections[missingConnection].parent1Connection1 = connections[missingConnection].parent2Connection2;
                        }
                        else
                        {
                            filledConnections[missingConnection].parent1Connection2 = connections[missingConnection].parent2Connection2;
                        }

                        if (missingConnections[missingConnection] == 1)
                        {
                            missingConnections.Remove(missingConnection);
                        }
                        else
                        {
                            missingConnections[missingConnection] -= 1;
                        }

                        if (filledConnections[connections[missingConnection].parent2Connection2].parent1Connection1 == -1)
                        {
                            filledConnections[connections[missingConnection].parent2Connection2].parent1Connection1 = missingConnection;
                        }
                        else
                        {
                            filledConnections[connections[missingConnection].parent2Connection2].parent1Connection2 = missingConnection;
                        }

                        if (missingConnections[connections[missingConnection].parent2Connection2] == 1)
                        {
                            missingConnections.Remove(connections[missingConnection].parent2Connection2);
                        }
                        else
                        {
                            missingConnections[connections[missingConnection].parent2Connection2] -= 1;
                        }
                    }
                }
            }
        }

        private void AddRemainingConnections(ref SortedList<int, TSPConnectionMap> filledConnections, ref SortedList<int, int> missingConnections)
        {
            #region Add remaining connections

            while (missingConnections.Count > 0)
            {
                int matrixIndex = missingConnections.First().Key;

                int endPoint1;
                int endPoint2;

                DetermineEndPoints(matrixIndex, filledConnections, out endPoint1, out endPoint2);

                int pendingRemoval = -1;

                List<int> remainingKeys = new List<int>();

                foreach (KeyValuePair<int, int> pair in missingConnections)
                {
                    remainingKeys.Add(pair.Key);
                }

                remainingKeys = genericFisherYatesShuff<int>(remainingKeys.ToList());

                foreach (int key in remainingKeys)
                {
                    if (key != matrixIndex && key != endPoint1 && key != endPoint2)
                    {
                        if (filledConnections[matrixIndex].parent1Connection1 == -1)
                        {
                            filledConnections[matrixIndex].parent1Connection1 = key;
                        }
                        else
                        {
                            filledConnections[matrixIndex].parent1Connection2 = key;
                        }

                        if (filledConnections[key].parent1Connection1 == -1)
                        {
                            filledConnections[key].parent1Connection1 = matrixIndex;
                        }
                        else
                        {
                            filledConnections[key].parent1Connection2 = matrixIndex;
                        }

                        pendingRemoval = key;
                        break;
                    }
                }

                if (pendingRemoval > 0)
                {
                    if (missingConnections[matrixIndex] == 1)
                    {
                        missingConnections.Remove(matrixIndex);
                    }
                    else
                    {
                        missingConnections[matrixIndex] -= 1;
                    }

                    if (missingConnections[pendingRemoval] == 1)
                    {
                        missingConnections.Remove(pendingRemoval);
                    }
                    else
                    {
                        missingConnections[pendingRemoval] -= 1;
                    }
                }
                else if (missingConnections.Count == 2)
                {
                    int first = missingConnections.First().Key;
                    int last = missingConnections.Last().Key;

                    if (filledConnections[first].parent1Connection1 == -1)
                    {
                        filledConnections[first].parent1Connection1 = last;
                    }
                    else
                    {
                        filledConnections[first].parent1Connection2 = last;
                    }

                    if (filledConnections[last].parent1Connection1 == -1)
                    {
                        filledConnections[last].parent1Connection1 = first;
                    }
                    else
                    {
                        filledConnections[last].parent1Connection2 = first;
                    }

                    missingConnections.Clear();
                }
            }

            #endregion
        }

        private void GenerateOffSpringList(ref TSPPath parent1, ref TSPPath parent2, ref SortedList<int, TSPConnectionMap> filledConnections, ref List<TSPPoint> offspringPoints)
        {
            int previousMatrixIndex = filledConnections.First().Key;
            int startIndex = previousMatrixIndex;
            offspringPoints.Add(parent1.points[filledConnections.First().Value.parent1Index]);

            int currentMatrixIndex = filledConnections.First().Value.parent1Connection1;

            while (currentMatrixIndex != startIndex)
            {
                offspringPoints.Add(parent1.points[filledConnections[currentMatrixIndex].parent1Index]);

                int tempCurrentMatrixIndex = currentMatrixIndex;

                if (filledConnections[currentMatrixIndex].parent1Connection1 == previousMatrixIndex)
                {
                    currentMatrixIndex = filledConnections[currentMatrixIndex].parent1Connection2;
                }
                else
                {
                    currentMatrixIndex = filledConnections[currentMatrixIndex].parent1Connection1;
                }

                previousMatrixIndex = tempCurrentMatrixIndex;
            }
        }

        #endregion

        private List<TSPPath> GeneratePopulation(int initialPopulation)
        {
            List<TSPPath> population = new List<TSPPath>();

            //Generate chromosomes
            for (int i = 0; i < initialPopulation; i++)
            {
                //Generate random chromosome for population
                List<TSPPoint> points = FisherYatesShuffle();

                //Calculate fitness (distance) of chromosome
                double distance = CalculateDistance(points, true);

                //Create new TSPPath for chromosome
                TSPPath path = new TSPPath(points, distance);

                //Add chromosome to population
                population.Add(path);
            }

            return population;
        }

        public double GeneticAlgorithm(out List<TSPPoint> shortestPoints, int mutationProbability, int populationSize, int iterations)
        {
            //Generate initial population
            List<TSPPath> initialPopulation = GeneratePopulation(populationSize);            

            //Sort chromosomes by fitness
            initialPopulation.Sort();

            //Iterations
            for (int i = 0; i < iterations; i++)
            {
                //Select pair of chromosomes for mating
                TSPPath parent1 = SelectParent(initialPopulation);
                TSPPath parent2 = SelectParent(initialPopulation);

                //Create pair of offspring chromosomes by applying genetic operators - crossover and mutation
                TSPPath offspring = GenerateOffspring(parent1, parent2, mutationProbability);

                initialPopulation.Add(offspring);

                initialPopulation.RemoveAt(initialPopulation.Count - 2);

                //Sort chromosomes by fitness
                initialPopulation.Sort();
            }

            shortestPoints = initialPopulation[0].points;

            return initialPopulation[0].Fitness();
        }

        public double GeneticAlgorithmWOC(out List<TSPPoint> shortestPoints, int mutationProbability, int populationSize, int iterations)
        {
            //Debug.WriteLine("In GeneticAlgorithmWOC");
            List<TSPPath> initialPopulation = GeneratePopulation(populationSize);

            initialPopulation.Sort();

            for (int i = 0; i < iterations; i++)
            {
                //Select the two parents
                TSPPath parent1 = SelectParent(initialPopulation);
                TSPPath parent2 = SelectParent(initialPopulation);

                //Generate a child through crossover and mutation
                TSPPath offspring = GenerateOffspring(parent1, parent2, mutationProbability);
                initialPopulation.Add(offspring); //The offspring is now part of the population
                initialPopulation.RemoveAt(initialPopulation.Count - 2);

                initialPopulation.Sort(); //sorting chromosomes by their fitness
            }

            //final sort based off fitness
            initialPopulation.Sort();

            //Now we iterate through offsprings generated by WOC approach
            for (int i = 0; i < iterations; i++)
            {
                //Select the two parents
                TSPPath parent1 = SelectParent(initialPopulation);
                TSPPath parent2 = SelectParent(initialPopulation);

                //Generate a child through crossover and mutation
                TSPPath offspring = GenerateOffSpringWOC(parent1, parent2);
                //TSPPath offspring = GenerateOffspring(parent1, parent2, mutationProbability);
                initialPopulation.Add(offspring); //The offspring is now part of the population
                initialPopulation.RemoveAt(initialPopulation.Count - 2);

                initialPopulation.Sort(); //sorting chromosomes by their fitness
            }

            shortestPoints = initialPopulation[0].points;
            return CalculateDistance(initialPopulation[0].points, true);
            //return CalculateDistance(initialPopulation[0].points, true);
        }

        // Source: http://social.msdn.microsoft.com/Forums/en-AU/csharpgeneral/thread/0e22aecc-91fe-4112-9350-d898dc6bf416?persist=True
        private double GetRandomDouble()
        {
            var bytes = new byte[12];
            this.cryptoProvider.GetBytes(bytes);
            var stringBuilder = new StringBuilder("0.");
            var numbers = bytes.Select(i => Convert.ToInt32((i * 100 / 255) / 10)).ToArray();

            foreach (var number in numbers)
            {
                stringBuilder.Append(number);
            }

            return Convert.ToDouble(stringBuilder.ToString());
        }

        // Source: https://stackoverflow.com/questions/38668748/how-to-generate-random-int-value-using-system-security-cryptography
        private int GetRandomInt(int minValue, int maxValue)
        {
            // Generate four random bytes
            byte[] four_bytes = new byte[4];
            this.cryptoProvider.GetBytes(four_bytes);

            // Convert the bytes to a UInt32
            UInt32 scale = BitConverter.ToUInt32(four_bytes, 0);

            // And use that to pick a random number >= min and < max
            return (int)(minValue + (maxValue - minValue) * (scale / (uint.MaxValue + 1.0)));
        }

        private void MutateOffspring(List<TSPPoint> points, int mutationProbability)
        {
            double mutation = Convert.ToDouble(mutationProbability);
            //Randomly decide if offspring is to be mutated based on mutation probability
            if (GetRandomDouble() < (mutation / 100.00))
            {
                int index1 = GetRandomInt(0, points.Count);
                int index2 = GetRandomInt(0, points.Count);

                TSPPoint temp = points[index1];
                points[index1] = points[index2];
                points[index2] = temp;
            }
        }

        // Randomly returns a chromosome using weighted fitness
        // Source: http://stackoverflow.com/questions/2772882/c-picking-a-random-item-based-on-probabilities
        private TSPPath SelectParent(List<TSPPath> population)
        {
            double probability = GetRandomDouble();

            int i = 1;
            double cumulativeProbability = 0;
            while (cumulativeProbability < probability)
            {
                double assignedProbability = (double)(2 * i) / (double)(population.Count * (population.Count + 1));
                cumulativeProbability += assignedProbability;
                i++;
            }

            int parentIndex = Math.Abs(population.Count - 1 - i);

            return population[parentIndex];
        }

        private double CalculateDistance(List<TSPPoint> points, bool closePath)
        {
            double distance = 0;

            for (int i = 1; i < points.Count; i++)
            {
                distance += this.matrix[points[i - 1].matrixIndex, points[i].matrixIndex];
            }

            if (closePath)
            {
                distance += this.matrix[points.First().matrixIndex, points.Last().matrixIndex];
            }

            return distance;
        }
        #endregion

        #region Permutation

        //The following two functions are implemented from: https://www.codeproject.com/Articles/43767/A-C-List-Permutation-Iterator

        public static void RotateRight(IList sequence, int count)
        {
            object tmp = sequence[count - 1];
            sequence.RemoveAt(count - 1);
            sequence.Insert(0, tmp);
        }

        public static IEnumerable<IList> Permutate(IList sequence, int count)
        {
            if (count == 1) yield return sequence;
            else
            {
                for (int i = 0; i < count; i++)
                {
                    foreach (var perm in Permutate(sequence, count - 1))
                        yield return perm;
                    RotateRight(sequence, count);
                }
            }
        }

        #endregion
    }
}
