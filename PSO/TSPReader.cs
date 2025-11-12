using System.Diagnostics.CodeAnalysis;
using System.Globalization;

namespace PSO;

public class TSPReader {
    private string tspFolder;
    private string outputFolder;
    private string problemName;
    
    private int dimension;
    private bool hasGeoCoords;
    private double[,] coords;

    public TSPReader(string tspFolder, string outputFolder, string problemName) {
        this.tspFolder = tspFolder;
        this.outputFolder = outputFolder;
        this.problemName = problemName;
        
        ReadFile();
    }

    [MemberNotNull(nameof(coords))]
    private void ReadFile() {
        string[] problem = File.ReadAllLines(tspFolder + problemName + ".tsp");

        hasGeoCoords = false;
        bool nodeSection = false;
        
        foreach (var line in problem) { // read meta-data and coords of cities
            if (nodeSection && !line.Trim().Equals("EOF") && line.Length > 0) {
                if (coords == null) throw new Exception("Header of tsp-file does not specify dimensions");

                string[] node = line.Trim().Split(' ');
                int id = int.Parse(node[0]) - 1;
                coords[id, 0] = double.Parse(node[1], CultureInfo.InvariantCulture);
                coords[id, 1] = double.Parse(node[2], CultureInfo.InvariantCulture);
                if (hasGeoCoords) {
                    coords[id, 0] = ToDecimalDegrees(coords[id, 0]);
                    coords[id, 1] = ToDecimalDegrees(coords[id, 1]);
                }
            }
            else if (line.StartsWith("DIMENSION")) {
                dimension = int.Parse(line.Split(' ').Last());
                coords = new double[dimension, 2];
            }
            else if (line.Equals("NODE_COORD_SECTION")) {
                nodeSection = true;
            }
            else if (line.StartsWith("EDGE_WEIGHT_TYPE") && line.Split(' ').Last().Equals("GEO")) {
                hasGeoCoords = true;
            }
        }
    }
    
    private const double RRR = 6378.388;
    public int[,] CalcDists() {
        int[,] dists = new int[dimension, dimension];

        for (int i = 0; i < dimension; i++) {
            for (int j = i; j < dimension; j++) {
                if (i == j) {
                    dists[i, j] = 0;
                }
                else {
                    if (hasGeoCoords) {
                        // according to http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/TSPFAQ.html
                        double q1 = Math.Cos(ToRadian(coords[i, 1]) - ToRadian(coords[j, 1]));
                        double q2 = Math.Cos(ToRadian(coords[i, 0]) - ToRadian(coords[j, 0]));
                        double q3 = Math.Cos(ToRadian(coords[i, 0]) + ToRadian(coords[j, 0]));
                        dists[i, j] = (int)(RRR * Math.Acos(0.5 * ((1.0 + q1) * q2 - (1.0 - q1) * q3)) + 1.0);
                    }
                    else {
                        // use rounded euclidean
                        double dx = coords[i, 0] - coords[j, 0];
                        double dy = coords[i, 1] - coords[j, 1];
                        dists[i, j] = (int)Math.Round(Math.Sqrt(dx * dx + dy * dy));
                    }

                    dists[j, i] = dists[i, j];
                }
            }
        }

        return dists;
    }

    private double ToDecimalDegrees(double degreesMinutes) {
        double deg = (int)degreesMinutes;
        return deg + (degreesMinutes - deg) * 100.0 / 60.0;
    }

    private double ToRadian(double degrees) {
        return Math.PI * degrees / 180.0;
    }

    public void DrawTour(int[] best) { // "draw" the tour as a dot-file
        double[,] coords = this.coords.Clone() as double[,];
        if (hasGeoCoords) {
            for (int i = 0; i < dimension; i++) { // switch latitude and longitude to be represented as x,y values
                (coords[i, 0], coords[i, 1]) = (coords[i, 1], coords[i, 0]);
            }
        }

        // search maximum extent to find good scaling factor
        double maxX = double.NegativeInfinity, minX = double.PositiveInfinity;
        double maxY = double.NegativeInfinity, minY = double.PositiveInfinity;
        for (int i = 0; i < dimension; i++) {
            if (coords[i, 0] < minX)
                minX = coords[i, 0];
            if (coords[i, 0] > maxX)
                maxX = coords[i, 0];
            
            if (coords[i, 1] < minY)
                minY = coords[i, 1];
            if (coords[i, 1] > maxY)
                maxY = coords[i, 1];
        }
        double scale = Math.Round(Math.Max(maxX - minX, maxY - minY) / 15, 2);
        
        
        string output = $"graph {problemName}{{\n\tgraph[layout=\"neato\"]\n\t";

        // list all cities and assign them correct coordinates
        for (int i = 0; i < dimension; i++) {
            output += $"{i+1}[pos=\"{coords[i, 0].ToString(CultureInfo.InvariantCulture)},{coords[i, 1].ToString(CultureInfo.InvariantCulture)}!\"]\n\t";
        }
        
        // connect cities via best path
        foreach (int c in best)
            output += $"{c+1} -- ";
        output += $"{best[0]+1}[penwidth={scale.ToString(CultureInfo.InvariantCulture)}]\n}}";
        
        File.WriteAllText(outputFolder + problemName + "_" + DateTime.Now.ToString("yyyyMMddHHmmss") + ".dot", output);
    }
}