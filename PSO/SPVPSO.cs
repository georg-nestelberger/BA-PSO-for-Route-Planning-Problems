namespace PSO;

public class SPVPSO(int[,] dists, int iterations, int swarmSize, double inertia = 1, double cognitiveCoefficient = 2, double socialCoefficient = 2) : PSO(dists, iterations, swarmSize) {
    private const double INIT_MAX_VALUE = 20;
    private const double MAX_VELOCITY = 20;

    private int dimensions = dists.GetLength(0);
    private double inertia = inertia;
    private double cognitiveCoefficient = cognitiveCoefficient;
    private double socialCoefficient = socialCoefficient;

    private double[][] particles;
    private double[][] velocities;
    private double[][] pBests;
    private double[] pBestsFitness;
    private double[] gBest;
    private double gBestFitness;
    

    protected override void UpdateParticle(int p) {
        for (int d = 0; d < dimensions; d++) {
            velocities[p][d] = Math.Clamp(
                inertia * velocities[p][d] +
                cognitiveCoefficient * rand.NextDouble() * (pBests[p][d] - particles[p][d]) +
                socialCoefficient * rand.NextDouble() * (gBest[d] - particles[p][d]),
                -MAX_VELOCITY,
                MAX_VELOCITY
            );

            particles[p][d] += velocities[p][d];
        }

        double fitness = EvalPosition(p);
        if (fitness < pBestsFitness[p]) {
            SetPBest(p, fitness);

            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
        }
    }
    
    protected override void InitializeSwarm() {
        particles = new double[swarmSize][];
        velocities = new double[swarmSize][];
        pBests = new double[swarmSize][];
        pBestsFitness = new double[swarmSize];
        gBest = new double[dimensions];
        gBestFitness = double.PositiveInfinity;

        for (int p = 0; p < swarmSize; p++) {
            particles[p] = new double[dimensions];
            pBests[p] = new double[dimensions];
            velocities[p] = new double[dimensions];

            for (int d = 0; d < dimensions; d++) {
                double pos = (rand.NextDouble() - 0.5) * 2 * INIT_MAX_VALUE;
                particles[p][d] = pos;
                pBests[p][d] = pos;

                double v = (rand.NextDouble() - 0.5) * MAX_VELOCITY;
                velocities[p][d] = v;
            }

            double fitness = EvalPosition(p);
            pBestsFitness[p] = fitness;
            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
        }
    }

    protected override int[] GetBestPosition() {
        return GetOrder(gBest);
    }

    protected override double GetBestFitness() {
        return gBestFitness;
    }

    private double EvalPosition(int p) {
        double fitness = 0;
        int[] order = GetOrder(particles[p]);

        for (int i = 1; i < dimensions; i++) {
            fitness += dists[order[i - 1], order[i]];
        }
        fitness += dists[order.Last(), order[0]];
        
        return fitness;
    }

    private int[] GetOrder(double[] p) {
        return p.Select((val, idx) => (val, idx)).OrderBy(e => e.val).Select(e => e.idx).ToArray();
    }

    private void SetPBest(int p, double fitness) {
        pBestsFitness[p] = fitness;
        for (int d = 0; d < dimensions; d++) {
            pBests[p][d] = particles[p][d];
        }
    }

    private void SetGBest(int p, double fitness) {
        gBestFitness = fitness;
        for (int d = 0; d < dimensions; d++) {
            gBest[d] = particles[p][d];
        }
    }
}