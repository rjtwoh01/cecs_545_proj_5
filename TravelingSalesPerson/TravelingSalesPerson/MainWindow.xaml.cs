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
    public partial class MainWindow : Window
    {
        private string tspFileName;
        private List<Point> tspPoints;
        private Canvas canvas;
        private Viewbox viewbox;
        private TSP tsp;
        private string type;
        private TSP bfsDfsTsp;
        private List<TSPConnection> tSPConnections;
        GeneticAlgorithmOptions geneticAlgorithmOptions;
        private List<TSPPath> overallSolutions;


        public MainWindow()
        {
            InitializeComponent();
            tspFileName = "";
            type = "";
            hideRunTime();
            hideSolveButton();
            hideType();
            tspPoints = new List<Point>();
            tSPConnections = new List<TSPConnection>();
            overallSolutions = new List<TSPPath>();
        }

        #region Data Points

        private void populatePoints()
        {
            List<Point> newPoints = new List<Point>();
            using (StreamReader stream = new StreamReader(tspFileName))
            {
                string fileLine;
                bool coordinates = false;

                while ((fileLine = stream.ReadLine()) != null)
                {
                    string[] parts = fileLine.Split(' ');
                    if (coordinates)
                    {
                        if (parts.Length >= 3)
                        {
                            Point newPoint = new Point(Convert.ToDouble(parts[1]), Convert.ToDouble(parts[2]));
                            tspPoints.Add(newPoint);
                            newPoints.Add(newPoint);
                            Debug.WriteLine(newPoint);
                        }
                    }
                    else if (fileLine == "NODE_COORD_SECTION")
                    {
                        coordinates = true;
                    }
                }
            }
            plotPoints(newPoints);
        }

        private void plotPoints(List<Point> points)
        {
            int city = 1; //we start at the first city
            tsp = new TSP(points);

            viewbox = new Viewbox();
            viewbox.HorizontalAlignment = HorizontalAlignment.Stretch;
            viewbox.VerticalAlignment = VerticalAlignment.Stretch;

            canvas = new Canvas();

            foreach (Point point in points)
            {
                Debug.WriteLine("City: " + city);
                Ellipse ellipse = new Ellipse();
                ellipse.Width = 4;
                ellipse.Height = 4;
                ellipse.Fill = Brushes.Red;
                ellipse.Stroke = Brushes.Black;

                ellipse.ToolTip = city + ": (" + point.X + "," + point.Y + ")";

                // Position point on canvas
                Canvas.SetLeft(ellipse, point.X + tsp.canvasOffset.X);
                Canvas.SetTop(ellipse, point.Y + tsp.canvasOffset.Y);

                canvas.Children.Add(ellipse);

                city++;
            }

            canvas.Height = tsp.maxPoint.Y - tsp.minPoint.Y + 80;
            canvas.Width = tsp.maxPoint.X - tsp.minPoint.X + 80;
            Debug.WriteLine(canvas.Height);
            Debug.WriteLine(canvas.Width);

            viewbox.Child = canvas;
            mainGrid.Children.Add(viewbox);

            Debug.WriteLine(mainGrid.Children[0]);

            //this.UpdateLayout();
            Debug.WriteLine("Finished populating points");
            
        }

        public void drawLines(List<Point> fastestRoute)
        {
            Ellipse ellipse = canvas.Children[0] as Ellipse;
            if (ellipse != null)
            {
                ellipse.Fill = Brushes.Green;
            }

            if (type == "bfs" || type == "dfs")
            {
                for (int i = 0; i < fastestRoute.Count(); i++)
                {
                    if ((i + 1) != fastestRoute.Count())
                    {
                        //(fastestRoute[i], fastestRoute[i + 1]);
                        Point p1 = fastestRoute[i];
                        Point p2 = fastestRoute[i + 1];
                        p1.X += tsp.canvasOffset.X + 2;
                        p1.Y += tsp.canvasOffset.Y + 2;
                        p2.X += tsp.canvasOffset.X + 2;
                        p2.Y += tsp.canvasOffset.Y + 2;

                        //pathLine.Points.Add(point);

                        Shape pathLine = DrawLinkArrow(p1, p2);

                        canvas.Children.Insert(0, pathLine);
                    }
                }
            }
            else
            {
                Polygon polyLine = new Polygon();
                polyLine.Stroke = Brushes.Black;

                for (int i = 0; i < fastestRoute.Count(); i++)
                {
                    if ((i + 1) != fastestRoute.Count())
                    {
                        //(fastestRoute[i], fastestRoute[i + 1]);
                        Point point = fastestRoute[i];
                        point.X += tsp.canvasOffset.X + 2;
                        point.Y += tsp.canvasOffset.Y + 2;

                        polyLine.Points.Add(point);
                    }
                }
                canvas.Children.Insert(0, polyLine);
            }
        }

        //Taken (and adapted based off comments) from (and adapted based off comments): https://stackoverflow.com/a/5203530
        private static Shape DrawLinkArrow(Point p1, Point p2) 
        {
            GeometryGroup lineGroup = new GeometryGroup();
            double theta = Math.Atan2((p2.Y - p1.Y), (p2.X - p1.X)) * 180 / Math.PI;

            PathGeometry pathGeometry = new PathGeometry();
            PathFigure pathFigure = new PathFigure();
            Point p = new Point(p1.X + ((p2.X - p1.X) / 1), p1.Y + ((p2.Y - p1.Y) / 1));
            pathFigure.StartPoint = p;

            Point lpoint = new Point(p.X + 3, p.Y + 5);
            Point rpoint = new Point(p.X - 3, p.Y + 5);
            LineSegment seg1 = new LineSegment();
            seg1.Point = lpoint;
            pathFigure.Segments.Add(seg1);

            LineSegment seg2 = new LineSegment();
            seg2.Point = rpoint;
            pathFigure.Segments.Add(seg2);

            LineSegment seg3 = new LineSegment();
            seg3.Point = p;
            pathFigure.Segments.Add(seg3);

            pathGeometry.Figures.Add(pathFigure);
            RotateTransform transform = new RotateTransform();
            transform.Angle = theta + 90;
            transform.CenterX = p.X;
            transform.CenterY = p.Y;
            pathGeometry.Transform = transform;
            lineGroup.Children.Add(pathGeometry);

            LineGeometry connectorGeometry = new LineGeometry();
            connectorGeometry.StartPoint = p1;
            connectorGeometry.EndPoint = p2;
            lineGroup.Children.Add(connectorGeometry);
            System.Windows.Shapes.Path path = new System.Windows.Shapes.Path();
            path.Data = lineGroup;
            path.StrokeThickness = 1;
            path.Stroke = Brushes.Black;

            return path;
        }
        
        public void setupBfsDfs()
        {
            tSPConnections.Clear();
            Polygon polyLine = new Polygon();
            polyLine.Stroke = Brushes.Black;

            //For this, we have to act on the assumption that the only file being provided is the 11 point one with the known (outside of the program) path
            for (int i = 0; i <= 10; i++)
            {
                TSPConnection tspConnect = new TSPConnection(tspPoints[i].X, tspPoints[i].Y);
                switch (i) { 
                    case 0:                        
                        tspConnect.connection1 = new Point(tspPoints[1].X, tspPoints[1].Y);
                        tspConnect.connection2 = new Point(tspPoints[2].X, tspPoints[2].Y);
                        tspConnect.connection3 = new Point(tspPoints[3].X, tspPoints[3].Y);
                        break;
                    case 1:
                        tspConnect.connection1 = new Point(tspPoints[2].X, tspPoints[2].Y);
                        break;
                    case 2:
                        tspConnect.connection1 = new Point(tspPoints[3].X, tspPoints[3].Y);
                        tspConnect.connection2 = new Point(tspPoints[4].X, tspPoints[4].Y);
                        break;
                    case 3:
                        tspConnect.connection1 = new Point(tspPoints[4].X, tspPoints[4].Y);
                        tspConnect.connection2 = new Point(tspPoints[5].X, tspPoints[5].Y);
                        tspConnect.connection3 = new Point(tspPoints[6].X, tspPoints[6].Y);
                        break;
                    case 4:
                        tspConnect.connection1 = new Point(tspPoints[6].X, tspPoints[6].Y);
                        tspConnect.connection2 = new Point(tspPoints[7].X, tspPoints[7].Y);
                        break;
                    case 5:
                        tspConnect.connection1 = new Point(tspPoints[7].X, tspPoints[7].Y);
                        break;
                    case 6:
                        tspConnect.connection1 = new Point(tspPoints[8].X, tspPoints[8].Y);
                        tspConnect.connection2 = new Point(tspPoints[9].X, tspPoints[9].Y);
                        break;
                    case 7:
                        tspConnect.connection1 = new Point(tspPoints[8].X, tspPoints[8].Y);
                        tspConnect.connection2 = new Point(tspPoints[9].X, tspPoints[9].Y);
                        tspConnect.connection3 = new Point(tspPoints[10].X, tspPoints[10].Y);
                        break;
                    case 8:
                        tspConnect.connection1 = new Point(tspPoints[10].X, tspPoints[10].Y);
                        break;
                    case 9:
                        tspConnect.connection1 = new Point(tspPoints[10].X, tspPoints[10].Y);
                        break;
                }
                tSPConnections.Add(tspConnect);
            }
            int city = 1;
            foreach(TSPConnection tspConnect in tSPConnections)
            {
                Debug.WriteLine(city + ":" + tspConnect);
                city++;
            }
        }

        public void solveGeneticAlgorithm(int crossoverPoint, int mutationProbability, int populationSize, int iterations, int trials, bool wocChecked)
        {
            overallSolutions.Clear();

            List<TSPPoint> shortestPoints;
            List<Point> drawPoints = new List<Point>();

            Stopwatch largeSW = Stopwatch.StartNew();

            for (int i = 0; i < trials; i++)
            {
                // Start stopwatch for calculating Genetic Algorithm execution time
                Stopwatch sw = Stopwatch.StartNew();

                // Solve TSP using Genetic Algorithm
                double shortestPath;
                if (!wocChecked)
                    shortestPath = tsp.GeneticAlgorithm(out shortestPoints, crossoverPoint, mutationProbability, populationSize, iterations);
                else
                    shortestPath = tsp.GeneticAlgorithmWOC(out shortestPoints, crossoverPoint, mutationProbability, populationSize, iterations);

                // Stop stopwatch for calculating Genetic Algorithm execution time
                sw.Stop();

                // Calculate Genetic Algorithm execution time
                TimeSpan elapsedTime = sw.Elapsed;

                TSPPath solution = new TSPPath(shortestPoints, shortestPath);
                solution.elapsedTime = elapsedTime;

                overallSolutions.Add(solution);
            }

            largeSW.Stop();
            TimeSpan elapsedTimeOverall = largeSW.Elapsed;

            TSPPath bestSolution = overallSolutions.Min();
            foreach (TSPPoint tSPPoint in bestSolution.points)
            {
                drawPoints.Add(tSPPoint.point);
            }

            drawLines(drawPoints);
            string shortestDistance = String.Format("{0:0.00}", bestSolution.Fitness());
            this.lblRunTime.Content = "Distance: " + shortestDistance + "\nRun Time: " + elapsedTimeOverall + ", solution: " + bestSolution.elapsedTime.ToString();

            displayRunTime();

            writeFile(trials, crossoverPoint, mutationProbability);
        }

        public void writeFile(int trials, int crossoverPoint, int mutationProbability)
        {
            try
            {
                using (StreamWriter writer = new StreamWriter("C:/temp/genetic_algorithm_trials-" + trials + "_crossover-" + crossoverPoint + "_mutation-" + mutationProbability + ".csv"))
                {
                    writer.WriteLine("Iteration,Fitness,Elapsed Time");

                    int i = 0;

                    foreach (TSPPath solution in overallSolutions)
                    {
                        writer.WriteLine(i + "," + solution.Fitness() + "," + solution.elapsedTime.ToString("G"));
                        i++;
                    }
                }
            }
            catch(IOException e) {
                MessageBox.Show("Please close the open file");
                Debug.WriteLine(e);
            }
        }

        #endregion

        #region UI Elements

        public void emptyCanvas()
        {
            if (canvas != null)
                this.canvas.Children.Clear();
            this.tspPoints.Clear();
        }

        public void hideSolveButton()
        {
            this.btnSolve.Visibility = Visibility.Hidden;
        }

        public void showSolveButton()
        {
            this.btnSolve.Visibility = Visibility.Visible;
        }

        public void displayRunTime()
        {
            this.lblRunTime.Visibility = Visibility.Visible;
            this.UpdateLayout();
        }

        public void hideRunTime()
        {
            this.lblRunTime.Visibility = Visibility.Hidden;
            this.UpdateLayout();
        }

        public void displayType()
        {
            this.btnSelectTSPType.Visibility = Visibility.Visible;
            this.UpdateLayout();
        }

        public void hideType()
        {
            this.btnSelectTSPType.Visibility = Visibility.Hidden;
            this.UpdateLayout();
        }

        #endregion

        #region UI Events

        //Slightly modified for my purposes from: https://stackoverflow.com/questions/10315188/open-file-dialog-and-select-a-file-using-wpf-controls-and-c-sharp
        private void btnFileUpload_Click(object sender, RoutedEventArgs e)
        {
            hideSolveButton();
            emptyCanvas();
            OpenFileDialog dlg = new OpenFileDialog();

            dlg.DefaultExt = ".tsp";
            dlg.Filter = "TSP Files|*.tsp";

            Nullable<bool> result = dlg.ShowDialog();
            if (result == true)
            {
                string fileName = dlg.FileName;
                Debug.WriteLine(fileName);
                this.tspFileName = fileName;
                populatePoints();
            }

            displayType();
            hideRunTime();            
        }


        private void btnSolve_Click(object sender, RoutedEventArgs e)
        {
            if (type == "bruteForce")
            {
                Stopwatch sw = Stopwatch.StartNew();
                List<Point> tempResult = tsp.BruteForce();
                sw.Stop();

                TimeSpan elapsedTime = sw.Elapsed;
                string shortestDistance = String.Format("{0:0.00}", tsp.shortestDistance);
                this.lblRunTime.Content = "Distance: " + shortestDistance + "\nRun Time: " + elapsedTime.ToString();

                displayRunTime();
                drawLines(tempResult);
            }
            else if (type == "dfs")
            {
                Stopwatch sw = Stopwatch.StartNew();
                List<Point> tempResult = tsp.DFS(tSPConnections);
                sw.Stop();

                TimeSpan elapsedTime = sw.Elapsed;
                string shortestDistance = String.Format("{0:0.00}", tsp.shortestDistance);
                this.lblRunTime.Content = "Distance: " + shortestDistance + "\nRun Time: " + elapsedTime.ToString();

                displayRunTime();
                drawLines(tempResult);
            }
            else if (type == "bfs")
            {
                Stopwatch sw = Stopwatch.StartNew();
                List<Point> tempResult = tsp.BFS(tSPConnections);
                sw.Stop();

                TimeSpan elapsedTime = sw.Elapsed;
                string shortestDistance = String.Format("{0:0.00}", tsp.shortestDistance);
                this.lblRunTime.Content = "Distance: " + shortestDistance + "\nRun Time: " + elapsedTime.ToString();

                displayRunTime();
                drawLines(tempResult);
            }
            else if (type == "closestEdge")
            {
                Stopwatch sw = Stopwatch.StartNew();
                List<Point> tempResult = tsp.ClosestEdgeInsertion();
                sw.Stop();

                TimeSpan elapsedTime = sw.Elapsed;
                string shortestDistance = String.Format("{0:0.00}", tsp.shortestDistance);
                this.lblRunTime.Content = "Distance: " + shortestDistance + "\nRun Time: " + elapsedTime.ToString();

                displayRunTime();
                drawLines(tempResult);
            }
            else if (type == "geneticAlgorithm")
            {
                if (this.geneticAlgorithmOptions == null)
                {
                    this.geneticAlgorithmOptions = new GeneticAlgorithmOptions(this);
                }

                this.geneticAlgorithmOptions.Show();
            }
            else
            {
                MessageBox.Show(type + " is not implemented yet");
            }
        }

        //Implented from: https://dotnetlearning.wordpress.com/2011/02/20/dropdown-menu-in-wpf/
        private void btnSelectTSPType_Click(object sender, RoutedEventArgs e)
        {
            (sender as Button).ContextMenu.PlacementTarget = (sender as Button);
            (sender as Button).ContextMenu.Placement = System.Windows.Controls.Primitives.PlacementMode.Bottom;
            (sender as Button).ContextMenu.IsOpen = true;
        }

        private void bruteForceClick(object sender, RoutedEventArgs e)
        {
            if (canvas != null)
                this.canvas.Children.Clear();
            if (tspPoints.Count() != 0)
                plotPoints(tspPoints);
            showSolveButton();
            type = "bruteForce";
            Debug.WriteLine(type);
        }

        private void bfsClick(object sender, RoutedEventArgs e)
        {
            if (canvas != null)
                this.canvas.Children.Clear();
            if (tspPoints.Count() != 0)
                plotPoints(tspPoints);
            showSolveButton();
            type = "bfs";
            Debug.WriteLine(type);
            setupBfsDfs();
        }

        private void dfsClick(object sender, RoutedEventArgs e)
        {
            if (canvas != null)
                this.canvas.Children.Clear();
            if (tspPoints.Count() != 0)
                plotPoints(tspPoints);
            showSolveButton();
            type = "dfs";
            Debug.WriteLine(type);
            setupBfsDfs();            
        }

        private void closestEdgeClick(object sender, RoutedEventArgs e)
        {
            if (canvas != null)
                this.canvas.Children.Clear();
            if (tspPoints.Count() != 0)
                plotPoints(tspPoints);
            showSolveButton();
            type = "closestEdge";
            Debug.WriteLine(type);
        }

        private void geneticAlgorithmClick(object sender, RoutedEventArgs e)
        {
            if (canvas != null)
                this.canvas.Children.Clear();
            if (tspPoints.Count() != 0)
                plotPoints(tspPoints);
            showSolveButton();
            type = "geneticAlgorithm";
            Debug.WriteLine(type);
        }

        #endregion
    }
}