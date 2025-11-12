namespace PSO;

public class C3DPSO(int[,] dists, int iterations, int swarmSize, double w = 0.6, double c1 = 1.5, double c2 = 2, double c3 = 2) : PSO(dists, iterations, swarmSize) {
    private const int MAX_OCCURENCE = 4;
    
    private int dimensions = dists.GetLength(0);
    private double w = w;
    private double c1 = c1;
    private double c2 = c2;
    private double c3 = c3;

    private int[][] particles; // represent position not as set for easier evaluating
    private int[][] pBests;
    private double[] pBestsFitness;
    private HashSet<Edge>[] velocities;
    private int[] gBest;
    private double gBestFitness;
    
    protected override void UpdateParticle(int p) {
        List<Edge> vNew = new List<Edge>();
        int[] cityOccurrences = new int[dimensions];
        var vGBest = CalcDiffBetweenPos(c2 * rand.NextDouble(), gBest, particles[p]);
        var vPBest = CalcDiffBetweenPos(c1 * rand.NextDouble(), pBests[p], particles[p]);

        foreach (Edge e in vGBest) {
            if (cityOccurrences[e.N1] < MAX_OCCURENCE && cityOccurrences[e.N2] < MAX_OCCURENCE) {
                vNew.Add(e);
                cityOccurrences[e.N1]++;
                cityOccurrences[e.N2]++;
            }
        }
        foreach (Edge e in vPBest) {
            if (cityOccurrences[e.N1] < MAX_OCCURENCE && cityOccurrences[e.N2] < MAX_OCCURENCE) {
                vNew.Add(e);
                cityOccurrences[e.N1]++;
                cityOccurrences[e.N2]++;
            }
        }
        foreach (Edge e in velocities[p]) {
            if (cityOccurrences[e.N1] < MAX_OCCURENCE && cityOccurrences[e.N2] < MAX_OCCURENCE) {
                e.Probability *= w;
                vNew.Add(e);
                cityOccurrences[e.N1]++;
                cityOccurrences[e.N2]++;
            }
        }
        velocities[p] = new HashSet<Edge>(vNew);
        
        List<Edge> allEdges = new List<Edge>(vNew);
        for (int i = 0; i < dimensions - 1; i++) {
            allEdges.Add(new Edge(c3 * rand.NextDouble(), particles[p][i], particles[p][i + 1]));
        }
        allEdges.Add(new Edge(c3 * rand.NextDouble(), particles[p][dimensions - 1], particles[p][0]));

        int edgesCount = 0; // to allow last edge to close cycle and maybe set last edge along with second-to-last
        int?[,] edges = new int?[dimensions, 2];
        foreach (Edge e in allEdges) {
            if (rand.NextDouble() < e.Probability && edgesCount < dimensions && edges[e.N1, 1] == null && edges[e.N2, 1] == null) {

                // check if edge is allowed in (doesn't close cycle prematurely)
                if (edges[e.N1, 0] != null && edges[e.N2, 0] != null) {
                    bool isLoop = false;
                    int? prev = e.N2;
                    int? cur = edges[e.N2, 0];
                    while (cur != null) {
                        if (cur == e.N1) {
                            isLoop = true;
                            break;
                        }
                        int? tmp = cur;
                        if (edges[(int) cur, 0] == prev)
                            cur = edges[(int) cur, 1];
                        else
                            cur = edges[(int) cur, 0];
                        prev = tmp;
                    }
                    if (isLoop) continue;
                }
                
                if (edges[e.N1, 0] == null)
                    edges[e.N1, 0] = e.N2;
                else
                    edges[e.N1, 1] = e.N2;
                
                if (edges[e.N2, 0] == null)
                    edges[e.N2, 0] = e.N1;
                else
                    edges[e.N2, 1] = e.N1;

                edgesCount++;

                // if (edgesCount == dim - 1) {
                //     int? n1 = null;
                //     int? n2 = null;
                //     for (int i = 0; i < dim; i++) {
                //         if (edges[i, 1] == null) {
                //             if (n1 == null)
                //                 n1 = i;
                //             else
                //                 n2 = i;
                //         }
                //
                //         if (n2 != null) break;
                //     }
                //     edges[(int) n1, 1] = n2;
                //     edges[(int) n2, 1] = n1;
                //     edgesCount++;
                // }
            }
        }

        for (int i = 0; i < dimensions; i++) {
            for (int j = 0; j < 2; j++) {
                if (edges[i, j] == null) {
                    double cost = Double.PositiveInfinity;
                    int? otherNode = null;
                    int? otherIdx = null;
                    for (int k = i + 1; k < dimensions; k++) {
                        for (int l = 0; l < 2; l++) {
                            if (edges[k, l] == null && dists[i, k] < cost) {
                                if (j == 1 && l == 1 && edges[k, 0] != null && edgesCount < dimensions - 1) {
                                    bool isLoop = false;
                                    int? prev = k;
                                    int? cur = edges[k, 0];
                                    while (cur != null) {
                                        if (cur == i) {
                                            isLoop = true;
                                            break;
                                        }
                                        int? tmp = cur;
                                        if (edges[(int) cur, 0] == prev)
                                            cur = edges[(int) cur, 1];
                                        else
                                            cur = edges[(int) cur, 0];
                                        prev = tmp;
                                    }
                                    if (isLoop) continue;
                                }
                                
                                cost = dists[i, k];
                                otherNode = k;
                                otherIdx = l;
                            }
                        }
                    }
                    edges[i, j] = otherNode;
                    edges[(int) otherNode, (int) otherIdx] = i;
                    edgesCount++;
                }
            }
        }

        int[] newPos = new int[dimensions];
        int? pr = 0;
        int? c = edges[0, 0];
        newPos[0] = 0;
        for (int i = 1; i < dimensions; i++) {
            newPos[i] = (int) c;
            int? tmp = c;
            if (edges[(int) c, 0] == pr)
                c = edges[(int) c, 1];
            else
                c = edges[(int) c, 0];
            pr = tmp;
        }

        particles[p] = newPos;
        
        double fitness = EvalPosition(p);
        if (fitness < pBestsFitness[p]) {
            SetPBest(p, fitness);

            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
        }
    }

    protected override void InitializeSwarm() {
        particles = new int[swarmSize][];
        pBests = new int[swarmSize][];
        pBestsFitness = new double[swarmSize];
        velocities = new HashSet<Edge>[swarmSize];
        gBest = new int[dimensions];
        gBestFitness = double.PositiveInfinity;

        for (int p = 0; p < swarmSize; p++) {
            particles[p] = new int[dimensions];
            pBests[p] = new int[dimensions];
            Dictionary<int, double> dims = new Dictionary<int, double>();
            for (int i = 0; i < dimensions; i++) {
                dims.Add(i, rand.NextDouble());
            }

            var order = (from entry in dims orderby entry.Value ascending select entry.Key).ToArray();

            particles[p] = order;
            
            double fitness = EvalPosition(p);
            SetPBest(p, fitness);
            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
            
            int velSize = rand.Next(1, 14);
            velocities[p] = new HashSet<Edge>();
            for (int i = 0; i < velSize; i++) {
                int n1 = rand.Next(0, dimensions);
                int n2 = rand.Next(0, dimensions);
                while (n1 == n2) n2 = rand.Next(0, dimensions);
                velocities[p].Add(new Edge(rand.NextDouble(), n1, n2));
            }
        }
    }

    protected override int[] GetBestPosition() {
        return gBest;
    }

    protected override double GetBestFitness() {
        return gBestFitness;
    }

    private double EvalPosition(int p) {
        double fitness = 0;

        for (int i = 1; i < dimensions; i++) {
            fitness += dists[particles[p][i - 1], particles[p][i]];
        }
        fitness += dists[particles[p][dimensions - 1], particles[p][0]];
        
        return fitness;
    }
    
    private void SetPBest(int p, double fitness) {
        pBestsFitness[p] = fitness;
        Array.Copy(particles[p], pBests[p], particles[p].Length);
    }

    private void SetGBest(int p, double fitness) {
        gBestFitness = fitness;
        Array.Copy(particles[p], gBest, particles[p].Length);
    }

    private ISet<Edge> CalcDiffBetweenPos(double prob, int[] p1, int[] p2) {
        HashSet<Edge> res = new  HashSet<Edge>();
        int[,] neighbors = new int[dimensions, 2];
        for (int i = 1; i < dimensions - 1; i++) {
            neighbors[p2[i], 0] = p2[i + 1];
            neighbors[p2[i], 1] = p2[i - 1];
        }
        neighbors[p2[0], 0] = p2[1];
        neighbors[p2[0], 1] = p2[dimensions-1];
        neighbors[p2[dimensions - 1], 0] = p2[0];
        neighbors[p2[dimensions - 1], 1] = p2[dimensions-2];

        for (int i = 0; i < dimensions - 1; i++) {
            if (neighbors[p1[i], 0] != p1[i + 1] && neighbors[p1[i], 1] != p1[i + 1]) {
                res.Add(new Edge(prob, p1[i], p1[i + 1]));
            }
        }
        if (neighbors[p1[dimensions-1], 0] != p1[0] && neighbors[p1[dimensions-1], 1] != p1[0]) {
            res.Add(new Edge(prob, p1[dimensions-1], p1[0]));
        }

        return res;
    }
    
    private class Edge(double prob, int n1, int n2) {
        public double Probability { get; set; } = prob;
        public int N1 { get; set; } = n1;
        public int N2 { get; set; } = n2;

        public Edge(int n1, int n2) : this(1.0, n1, n2) { }
    }
}