namespace PSO;

public class SSPSO3opt(int[,] dists, int iterations, int swarmSize) : SSPSO(dists, iterations, swarmSize) {

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
        

        // --- 3-opt ---
        Node start = new Node(newPos[0]);
        Node prev = start;
        for (int i = dimensions - 1; i > 0; i--) {
            Node tmp = new Node(newPos[i], prev);
            prev.N2 = tmp;
            prev = tmp;
        }
        start.N1 = prev;
        prev.N2 = start;
        
        // pick 3 nodes to delete a connection from
        List<int> nums = new List<int>();
        for (int i = 0; i < 3; i++) nums.Add(rand.Next(0, dimensions));
        nums.Sort();
        
        List<Node> nodes = new List<Node>();
        double prevCosts = 0;
        Node c = start;
        for (int i = 0; i <= nums[2]; i++) {
            if (nums.Contains(i)) {
                Node pr = c.N2;
                pr.N1 = null;
                c.N2 = null;
                nodes.Add(pr);
                nodes.Add(c);
                prevCosts += dists[pr.Id, c.Id];
            }
            c = c.N1;
        }
        
        // remove duplicates
        for (int i = 0; i < nodes.Count - 1; i++) {
            if (nodes[i] == nodes[i + 1]) {
                nodes.RemoveAt(i + 1);
                i--;
            }
        }
        
        int[]? improvedPos = null;
        Node X1 = nodes.First(); // first node in list must never be connected with last node or multiple cycles are created
        Node X2, Y1, Y2, Z1;
        Node Z2 = nodes.Last();
        for (int i = 1; i < nodes.Count - 1; i++) { // try to connect first node with any other but the last
            X2 = nodes[i];
            X1.AddNeighbor(X2);
            X2.AddNeighbor(X1);
        
            Y1 = X2; // look for next opening in cycle
            Node prevNode = X1;
            while (Y1.Next(prevNode) != null) {
                Node tmp = Y1;
                Y1 = Y1.Next(prevNode);
                prevNode = tmp;
            }
            
            // test all possible (i.e. not first, last, same as X2 or itself) nodes to connect opening to
            for (int j = 1; j < nodes.Count - 1; j++) {
                if (i == j || Y1 == nodes[j]) continue;
                Y2 = nodes[j];
                Y1.AddNeighbor(Y2);
                Y2.AddNeighbor(Y1);
                
                Z1 = Y2;
                for (int k = 1; k < nodes.Count - 1; k++) { // Z1 must either be same as Y2 or the last remaining "unused" value nodes 
                    Node n = nodes[k];
                    if (X2 != n && Y1 != n && Y2 != n)
                        Z1 = n;
                }
                // Z1 can only be connected to Z2, no need for trying different possibilities
                Z1.AddNeighbor(Z2);
                Z2.AddNeighbor(Z1);

                // calc new costs and update when better
                double newCosts = dists[X1.Id, X2.Id] + dists[Y1.Id, Y2.Id] + dists[Z1.Id, Z2.Id];
                if (newCosts < prevCosts) {
                    prevCosts = newCosts;
                    improvedPos = new int[dimensions];
                    
                    Node curr = start;
                    prevNode = curr.N2;
                    for (int k = 0; k < dimensions; k++) {
                        improvedPos[k] = curr.Id;
                        Node tmp = curr;
                        curr = curr.Next(prevNode);
                        prevNode = tmp;
                    }
                }
                
                // undo all connections to try with other permutations
                Z1.RemoveNeighbor(Z2);
                Z2.RemoveNeighbor(Z1);
                Y1.RemoveNeighbor(Y2);
                Y2.RemoveNeighbor(Y1);
            }
            X1.RemoveNeighbor(X2);
            X2.RemoveNeighbor(X1);
        }
        
        int velSize = rand.Next(1, MAX_INITIAL_VELOCITY_LENGTH + 1); // randomize velocity
        velocities[p] = new int[2*velSize];
        for (int i = 0; i < 2 * velSize; i++) {
            velocities[p][i] = rand.Next(0, dimensions);
        }
        
        particles[p] = improvedPos ?? newPos;
        
        double fitness = EvalPosition(p);
        if (fitness < pBestsFitness[p]) {
            SetPBest(p, fitness);

            if (fitness < gBestFitness) {
                SetGBest(p, fitness);
            }
        }
    }
}

class Node {
    public int Id { get; }
    public Node? N1 { get; set; }
    public Node? N2 { get; set; }

    public Node(int id, Node? n1) {
        Id = id;
        N1 = n1;
    }
    
    public Node(int id) : this(id, null) {}

    public Node? Next(Node n) {
        if (n == N1) return N2;
        if (n == N2) return N1;
        throw new Exception("No");
    }

    public void AddNeighbor(Node n) {
        if (N1 == null) N1 = n;
        else if (N2 == null) N2 = n;
    }
    
    public void RemoveNeighbor(Node n) {
        if (N1 == n) N1 = null;
        else if (N2 == n) N2 = null;
    }
}