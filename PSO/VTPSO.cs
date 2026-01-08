namespace PSO;

public class VTPSO(int[,] dists, int iterations, int swarmSize) : SSPSO(dists, iterations, swarmSize) {

    protected override void UpdateParticle(int p) {
        List<int> diffPB = CalcBSSBetweenPoints(pBests[p], particles[p]);
        List<int> diffGB = CalcBSSBetweenPoints(gBest, particles[p]);
        List<int> newV = new List<int>(velocities[p]);

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

        // velocity tentative
        double newFitness = EvalPosition(p);
        double tentFitness = newFitness;
        int[] newPos = new int[dimensions];
        int[] tentPos = new int[dimensions];
        Array.Copy(particles[p], newPos, dimensions);
        Array.Copy(particles[p], tentPos, dimensions);
        for (int i = 0; i < newV.Count; i += 2) { // iterate over every swap-pair in the velocity, calc the new fitness and choose the best one
            if (Math.Abs(newV[i] - newV[i+1]) != 1 && Math.Abs(newV[i] - newV[i+1]) != dimensions-1) { // if cities to swap are not neighbors
                tentFitness = tentFitness
                              - dists[tentPos[(newV[i] - 1 + dimensions) % dimensions], tentPos[newV[i]]] // remove dists from first city to its neighbors
                              - dists[tentPos[newV[i]], tentPos[(newV[i] + 1) % dimensions]]
                              + dists[tentPos[(newV[i + 1] - 1 + dimensions) % dimensions], tentPos[newV[i]]] // add dists from first cities neighbors to second city
                              + dists[tentPos[newV[i]], tentPos[(newV[i + 1] + 1) % dimensions]]
                              - dists[tentPos[(newV[i + 1] - 1 + dimensions) % dimensions], tentPos[newV[i + 1]]] // remove dists from second city to its neighbors
                              - dists[tentPos[newV[i + 1]], tentPos[(newV[i + 1] + 1) % dimensions]]
                              + dists[tentPos[(newV[i] - 1 + dimensions) % dimensions], tentPos[newV[i + 1]]] // add dists from second cities neighbors to first city
                              + dists[tentPos[newV[i + 1]], tentPos[(newV[i] + 1) % dimensions]];
            }
            else { // cities to be swapped are neighbors
                if ((newV[i] < newV[i + 1] || (newV[i] == dimensions-1 && newV[i+1] == 0)) && (newV[i] != 0 || newV[i+1] != dimensions-1)) { // first city is to the left of the second city
                    tentFitness = tentFitness
                                  - dists[tentPos[(newV[i] - 1 + dimensions) % dimensions], tentPos[newV[i]]]
                                  - dists[tentPos[newV[i + 1]], tentPos[(newV[i + 1] + 1) % dimensions]]
                                  + dists[tentPos[(newV[i] - 1 + dimensions) % dimensions], tentPos[newV[i + 1]]]
                                  + dists[tentPos[newV[i]], tentPos[(newV[i + 1] + 1) % dimensions]];
                }
                else { // first city is to the right of the second city
                    tentFitness = tentFitness
                                  - dists[tentPos[newV[i]], tentPos[(newV[i] + 1) % dimensions]]
                                  - dists[tentPos[(newV[i + 1] - 1 + dimensions) % dimensions], tentPos[newV[i + 1]]]
                                  + dists[tentPos[newV[i + 1]], tentPos[(newV[i] + 1) % dimensions]]
                                  + dists[tentPos[(newV[i + 1] - 1 + dimensions) % dimensions], tentPos[newV[i]]];
                }
            }

            (tentPos[newV[i]], tentPos[newV[i+1]]) = (tentPos[newV[i+1]], tentPos[newV[i]]); // do the actual swap
            if (tentFitness < newFitness) {
                newFitness = tentFitness;
                Array.Copy(tentPos, newPos, dimensions);
            }
        }

        if (newFitness < pBestsFitness[p]) { // if there is a new pBest also do position tentative
            List<int> pos = new List<int>(newPos);
            for (int i = 1; i < pos.Count; i++) {
                int node = pos[i];
                int best = i;
                double tmpFitness = newFitness - dists[pos[i-1], node] - dists[node, pos[(i+1)%dimensions]] + dists[pos[i-1], pos[(i+1)%dimensions]];
                pos.RemoveAt(i);
                for (int j = 0; j < pos.Count; j++) {
                    tentFitness = tmpFitness - dists[pos[(j - 1 + pos.Count) % pos.Count], pos[j]] + dists[pos[(j - 1 + pos.Count) % pos.Count], node] + dists[node, pos[j]];
                    if (tentFitness < newFitness) {
                        best = j;
                        newFitness = tentFitness;
                    }
                }
                pos.Insert(best, node);
            }
            
            int kMax = dimensions/2;
            int k = rand.Next(2, kMax + 1);
            for (int i = 1; i < pos.Count + 1 - k; i++) {
                List<int> nodes = new List<int>();
                for (int j = 0; j < k; j++) 
                    nodes.Add(pos[i+j]);
                int best = i;
                double tmpFitness = newFitness-dists[pos[i-1], pos[i]] - dists[pos[i+k-1], pos[(i+k)%dimensions]] + dists[pos[i-1], pos[(i+k)%dimensions]];
                pos.RemoveRange(i, k);
                for (int j = 0; j < pos.Count; j++) {
                    tentFitness = tmpFitness - dists[pos[(j - 1 + pos.Count) % pos.Count], pos[j]] + dists[pos[(j - 1 + pos.Count) % pos.Count], nodes.First()] + dists[nodes.Last(), pos[j]];
                    if (tentFitness < newFitness) {
                        best = j;
                        newFitness = tentFitness;
                    }
                }
                bool reversed = false;
                nodes.Reverse();
                for (int j = 0; j < pos.Count; j++) {
                    tentFitness = tmpFitness - dists[pos[(j - 1 + pos.Count) % pos.Count], pos[j]] + dists[pos[(j - 1 + pos.Count) % pos.Count], nodes.First()] + dists[nodes.Last(), pos[j]];
                    if (tentFitness < newFitness) {
                        reversed = true;
                        best = j;
                        newFitness = tentFitness;
                    }
                }
                if (!reversed) nodes.Reverse();
                pos.InsertRange(best, nodes);
            }
            newPos = pos.ToArray();
        }
        
        particles[p] = newPos;
        double fitness = EvalPosition(p);
        if (Math.Abs(newFitness - fitness) > 1) {
            Console.WriteLine("B");
        }

        velocities[p] = CalcBSSBetweenPoints(newPos, particles[p]).ToArray();
        
        if (fitness < pBestsFitness[p]) {
            SetPBest(p, fitness);

            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
        }
    }
}