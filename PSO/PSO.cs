namespace PSO;

public abstract class PSO(int[,] dists, int iterations, int swarmSize) : IPSO {
    protected int[,] dists = dists;
    protected int iterations = iterations;
    protected int swarmSize = swarmSize;
    
    protected int currentIteration;

    protected Random rand = new Random();

    
    public virtual double Run(out int[] best) {
        InitializeSwarm();

        for (currentIteration = 0; currentIteration < iterations; currentIteration++) {
            // if (currentIteration % 100 == 0) {
            //     Console.WriteLine("Iteration " + currentIteration + ": " + GetBestFitness());
            // }

            for (int p = 0; p < swarmSize; p++) {
                UpdateParticle(p);
            }
        }

        best = GetBestPosition();
        return GetBestFitness();
    }

    protected abstract void InitializeSwarm();
    protected abstract void UpdateParticle(int p);
    protected abstract int[] GetBestPosition();
    protected abstract double GetBestFitness();
}