namespace PSO;

public class IESTPSO(int[,] dists, int iterations, int swarmSize) : SSPSO(dists, iterations, swarmSize) {

    protected override void UpdateParticle(int p) {
        List<int> diffPB = CalcBSSBetweenPoints(pBests[p], particles[p]);
        List<int> diffGB = CalcBSSBetweenPoints(gBest, particles[p]);
        List<int> newV = new List<int>();

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
            (newPos[newV[i]], newPos[newV[i + 1]]) = (newPos[newV[i + 1]], newPos[newV[i]]);
        }
        
        particles[p] = newPos;
        double fitness = EvalPosition(p);

        double tentFitness = fitness;
        // try removing 1 city from the sequence and inserting it anywhere else
        List<int> pos = new List<int>(newPos);
        for (int i = 1; i < pos.Count; i++) {
            int node = pos[i];
            int best = i;
            double tmpFitness = tentFitness - dists[pos[i-1], node] - dists[node, pos[(i+1)%dimensions]] + dists[pos[i-1], pos[(i+1)%dimensions]];
            pos.RemoveAt(i);
            for (int j = 0; j < pos.Count; j++) {
                double newFitness = tmpFitness - dists[pos[(j - 1 + pos.Count) % pos.Count], pos[j]] + dists[pos[(j - 1 + pos.Count) % pos.Count], node] + dists[node, pos[j]];
                if (newFitness < tentFitness) {
                    best = j;
                    tentFitness = newFitness;
                }
            }
            pos.Insert(best, node);
        }

        // try moving 2 to kMax cities in the sequence
        int kMax;
        if (currentIteration / (double) iterations < 0.3)
            kMax = (int)Math.Ceiling(dimensions / 10.0);
        else if (currentIteration / (double) iterations < 0.65)
            kMax = (int)Math.Ceiling(dimensions / 5.0);
        else
            kMax = (int)Math.Ceiling(dimensions / 3.0);
        int k = rand.Next(2, kMax + 1);
        for (int i = 1; i < pos.Count + 1 - k; i++) {
            List<int> nodes = new List<int>();
            for (int j = 0; j < k; j++) 
                nodes.Add(pos[i+j]);
            int best = i;
            double tmpFitness = tentFitness-dists[pos[i-1], pos[i]] - dists[pos[i+k-1], pos[(i+k)%dimensions]] + dists[pos[i-1], pos[(i+k)%dimensions]];
            pos.RemoveRange(i, k);
            for (int j = 0; j < pos.Count; j++) {
                double newFitness = tmpFitness - dists[pos[(j - 1 + pos.Count) % pos.Count], pos[j]] + dists[pos[(j - 1 + pos.Count) % pos.Count], nodes.First()] + dists[nodes.Last(), pos[j]];
                if (newFitness < tentFitness) {
                    best = j;
                    tentFitness = newFitness;
                }
            }
            bool reversed = false; // also try inserting the moved sequence in reverse
            nodes.Reverse();
            for (int j = 0; j < pos.Count; j++) {
                double newFitness = tmpFitness - dists[pos[(j - 1 + pos.Count) % pos.Count], pos[j]] + dists[pos[(j - 1 + pos.Count) % pos.Count], nodes.First()] + dists[nodes.Last(), pos[j]];
                if (newFitness < tentFitness) {
                    reversed = true;
                    best = j;
                    tentFitness = newFitness;
                }
            }
            if (!reversed) nodes.Reverse();
            pos.InsertRange(best, nodes);
        }
        
        particles[p] = pos.ToArray();
        fitness = EvalPosition(p);
        
        if (fitness < pBestsFitness[p]) {
            SetPBest(p, fitness);

            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
        }
    }
}