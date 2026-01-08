using PSO;

const string TSP_DIR = @"D:\Schule\FH\BA\Code\TSP\";
const string DOT_DIR = @"D:\Schule\FH\BA\Code\dot\";

const string problem = "berlin52";
const int runs = 20;

TSPReader tspReader = new TSPReader(TSP_DIR, DOT_DIR, problem);
var dists = tspReader.CalcDists();
double bestLength = Double.PositiveInfinity;
int[] bestSolution = null;
double[] allLengths = new double[runs];
TimeSpan[] allTimes = new TimeSpan[runs];

for (int i = -1; i < runs; i++) { // start with i < 0 to allow for some warm-up runs
    var start = DateTime.Now;
    // IPSO pso = new SPVPSO(dists, 5000, 100, 0.5, 1.4, 1.4);
    // IPSO pso = new SSPSO(dists, 5000, 100);
    // IPSO pso = new SSPSO3opt(dists, 5000, 50);
    // IPSO pso = new IESTPSO(dists, 500, 20);
    // IPSO pso = new VTPSO(dists, 5000, 50);
    IPSO pso = new C3DPSO(dists, 5000, 30, 0.3, 1, 2.5, 2);
    double length = pso.Run(out var best);
    var timeTaken = DateTime.Now - start;
    
    if (i < 0) continue;
    
    Console.WriteLine($"Length: {length}  -  Time taken: {timeTaken}");
    Console.Write("Solution: ");
    foreach (var b in best) Console.Write($"{b + 1}, ");
    Console.WriteLine();
    Console.WriteLine();

    allLengths[i] = length;
    allTimes[i] = timeTaken;
    if (length < bestLength) {
        bestLength = length;
        bestSolution = best;
    }
}

double avg = allLengths.Average();
double avgMSec = allTimes.Average(t => t.TotalMilliseconds);
Console.WriteLine($"Average length: {avg}, Standard deviation: {Math.Sqrt(allLengths.Sum(l => Math.Pow(l - avg, 2)) / (runs - 1))}, Best length: {bestLength}");
Console.WriteLine($"Average duration: {TimeSpan.FromMilliseconds(avgMSec)}, Standard deviation: {TimeSpan.FromMilliseconds(Math.Sqrt(allTimes.Sum(t => Math.Pow(t.TotalMilliseconds - avgMSec, 2)) / (runs - 1)))}, Shortest duration: {allTimes.Min()}");

tspReader.DrawTour(bestSolution!);