namespace PSO;

public class SSPSO(int[,] dists, int iterations, int swarmSize) : PSO(dists, iterations, swarmSize) {
    protected const int MAX_INITIAL_VELOCITY_LENGTH = 5;
    
    protected int dimensions = dists.GetLength(0);
    protected int iterations = iterations;
    protected int swarmSize = swarmSize;

    protected int[][] particles;
    protected int[][] pBests;
    protected double[] pBestsFitness;
    protected int[][] velocities;
    protected int[] gBest;
    protected double gBestFitness;
    

    protected override void UpdateParticle(int p) {
        List<int> diffPB = CalcBSSBetweenPoints(pBests[p], particles[p]);
        List<int> diffGB = CalcBSSBetweenPoints(gBest, particles[p]);
        List<int> newV =  new List<int>(velocities[p]);

        double alpha = rand.NextDouble();
        for (int i = 0; i < diffPB.Count; i+=2) {
            if (rand.NextDouble() <= alpha) {
                newV.Add(diffPB[i]);
                newV.Add(diffPB[i+1]);
            }
        }
        
        double beta = rand.NextDouble();
        for (int i = 0; i < diffGB.Count; i+=2) {
            if (rand.NextDouble() <= beta) {
                newV.Add(diffGB[i]);
                newV.Add(diffGB[i+1]);
            }
        }

        int[] newPos = new int[dimensions];
        Array.Copy(particles[p], newPos, dimensions);
        for (int i = 0; i < newV.Count; i += 2) {
            (newPos[newV[i]], newPos[newV[i+1]]) = (newPos[newV[i+1]], newPos[newV[i]]);
        }

        // velocities[p] = CalcBSSBetweenPoints(newPos, particles[p]).ToArray(); // correct but worse?
        int velSize = rand.Next(1, MAX_INITIAL_VELOCITY_LENGTH + 1); // randomize velocity
        velocities[p] = new int[2*velSize];
        for (int i = 0; i < 2 * velSize; i++) {
            velocities[p][i] = rand.Next(0, dimensions);
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
        velocities = new int[swarmSize][];
        gBest = new int[dimensions];
        gBestFitness = double.PositiveInfinity;

        for (int p = 0; p < swarmSize; p++) {
            particles[p] = new int[dimensions];
            pBests[p] = new int[dimensions];
            
            Dictionary<int, double> dims = new Dictionary<int, double>();
            for (int i = 0; i < dimensions; i++) {
                dims.Add(i, rand.NextDouble());
            }
            particles[p] = (from entry in dims orderby entry.Value ascending select entry.Key).ToArray();
            
            double fitness = EvalPosition(p);
            SetPBest(p, fitness);
            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }

            int velSize = rand.Next(1, MAX_INITIAL_VELOCITY_LENGTH + 1);
            velocities[p] = new int[2*velSize];
            for (int i = 0; i < 2 * velSize; i++) {
                velocities[p][i] = rand.Next(0, dimensions);
            }
        }
    }

    protected override int[] GetBestPosition() {
        return gBest;
    }

    protected override double GetBestFitness() {
        return gBestFitness;
    }

    protected List<int> CalcBSSBetweenPoints(int[] p1, int[] p2) {
        int[] tmp = new int[dimensions];
        Array.Copy(p2, tmp, dimensions);
        p2 = tmp;
        List<int> swaps = new List<int>();
        for (int i = 0; i < dimensions; i++) {
            if (p1[i] != p2[i]) {
                swaps.Add(i);
                for (int j = i+1; j < dimensions; j++) {
                    if (p1[i] != p2[j]) continue;
                    swaps.Add(j);
                    (p2[i], p2[j]) = (p2[j], p2[i]);
                    break;
                }
            }
        }
        return swaps;
    }
    
    protected double EvalPosition(int p) {
        double fitness = 0;

        for (int i = 1; i < dimensions; i++) {
            fitness += dists[particles[p][i - 1], particles[p][i]];
        }
        fitness += dists[particles[p][dimensions - 1], particles[p][0]];
        
        return fitness;
    }
    
    protected void SetPBest(int p, double fitness) {
        pBestsFitness[p] = fitness;
        Array.Copy(particles[p], pBests[p], particles[p].Length);
    }

    protected void SetGBest(int p, double fitness) {
        gBestFitness = fitness;
        Array.Copy(particles[p], gBest, particles[p].Length);
    }
}