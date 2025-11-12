using PSO;

const string TSP_DIR = @"D:\Schule\FH\BA\Code\TSP\";
const string DOT_DIR = @"D:\Schule\FH\BA\Code\dot\";

string problem = "ulysses22";

TSPReader tspReader = new TSPReader(TSP_DIR, DOT_DIR, problem);
double bestLength = Double.PositiveInfinity;
int[] bestSolution = null;

for (int i = 0; i < 10; i++) {
    var dists = tspReader.CalcDists();
    var start = DateTime.Now;
    IPSO pso = new C3DPSO(dists, 1000, 100);
    double length = pso.Run(out var best);
    Console.WriteLine($"Length: {length}  -  Time taken: {DateTime.Now - start}");
    Console.Write("Solution: ");
    foreach (var b in best) Console.Write($"{b + 1}, ");
    Console.WriteLine();
    Console.WriteLine();

    if (length < bestLength) {
        bestLength = length;
        bestSolution = best;
    }
}

tspReader.DrawTour(bestSolution!);