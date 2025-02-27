#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <limits>
#include <algorithm>

using namespace std;

struct Edge {
    int to;
    int cost;
};

struct Node {
    int id;
    map<int, Edge> edges; // adjacency
};

void parseTopologyFile(const string& filename, map<int, Node>& nodes);
void parseMessageFile(const string& filename, vector<string>& messages);
void parseChangesFile(const string& filename, vector<tuple<int,int,int>>& changes);
void writeForwardingTable(ofstream& outfile, const map<int, map<int,int>>& dist,
                          const map<int, map<int,int>>& nextHop);
void runDistanceVector(map<int, Node>& nodes, map<int, map<int,int>>& dist,
                       map<int, map<int,int>>& nextHop);
void writeMessages(ofstream& outfile, const vector<string>& messages,
                   const map<int, map<int,int>>& nextHop,
                   const map<int, map<int,int>>& dist);

static const int INF = numeric_limits<int>::max();

int main(int argc, char** argv) {
    if (argc != 4) {
        printf("Usage: ./distvec topofile messagefile changesfile\n");
        return -1;
    }

    map<int, Node> nodes;
    parseTopologyFile(argv[1], nodes);

    vector<string> messages;
    parseMessageFile(argv[2], messages);

    vector<tuple<int,int,int>> changes;
    parseChangesFile(argv[3], changes);

    // Initialize dist and nextHop for each node
    map<int, map<int,int>> dist, nextHop;
    for (auto& [id, _] : nodes) {
        for (auto& [id2, _2] : nodes) {
            dist[id][id2] = (id == id2) ? 0 : INF;
            nextHop[id][id2] = (id == id2) ? id : -1;
        }
        for (auto& [neighbor, edge] : nodes[id].edges) {
            dist[id][neighbor] = edge.cost;
            nextHop[id][neighbor] = neighbor;
        }
    }

    // Run Distance Vector once initially
    runDistanceVector(nodes, dist, nextHop);

    // Write output
    ofstream outfile("output.txt");
    writeForwardingTable(outfile, dist, nextHop);
    writeMessages(outfile, messages, nextHop, dist);

    // Apply changes
    for (auto& [from, to, cost] : changes) {
        if (cost == -999) {
            nodes[from].edges.erase(to);
            nodes[to].edges.erase(from);
        } else {
            nodes[from].edges[to] = {to, cost};
            nodes[to].edges[from] = {from, cost};
        }
        // Re-init dist/nextHop after each change
        for (auto& [id, _] : nodes) {
            for (auto& [id2, _2] : nodes) {
                dist[id][id2] = (id == id2) ? 0 : INF;
                nextHop[id][id2] = (id == id2) ? id : -1;
            }
            for (auto& [neighbor, edge] : nodes[id].edges) {
                dist[id][neighbor] = edge.cost;
                nextHop[id][neighbor] = neighbor;
            }
        }
        // Re-run DV
        runDistanceVector(nodes, dist, nextHop);

        outfile << "----- At this point, change is applied\n";
        writeForwardingTable(outfile, dist, nextHop);
        writeMessages(outfile, messages, nextHop, dist);
    }
    outfile.close();

    return 0;
}

void parseTopologyFile(const string& filename, map<int, Node>& nodes) {
    ifstream infile(filename);
    string line;
    while (getline(infile, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        int from, to, cost;
        if (iss >> from >> to >> cost) {
            // Insert if missing
            nodes[from].id = from;
            nodes[to].id = to;
            nodes[from].edges[to] = {to, cost};
            nodes[to].edges[from] = {from, cost};
        }
    }
}

void parseMessageFile(const string& filename, vector<string>& messages) {
    ifstream infile(filename);
    string line;
    while (getline(infile, line)) {
        if (line.empty()) continue;
        messages.push_back(line);
    }
}

void parseChangesFile(const string& filename, vector<tuple<int,int,int>>& changes) {
    ifstream infile(filename);
    string line;
    while (getline(infile, line)) {
        if (line.empty()) continue;
        istringstream iss(line);
        int from, to, cost;
        if (iss >> from >> to >> cost) {
            changes.emplace_back(from, to, cost);
        }
    }
}

// Distance Vector core
void runDistanceVector(map<int, Node>& nodes,
                       map<int, map<int,int>>& dist,
                       map<int, map<int,int>>& nextHop)
{
    bool updated = true;
    int nodeCount = (int)nodes.size();

    // We do at most nodeCount-1 iterations to converge (standard Bellman-Ford).
    for (int _iter = 0; _iter < nodeCount - 1 && updated; _iter++) {
        updated = false;
        // For each node i, for each neighbor j, for each possible destination k
        for (auto& [i, nodeI] : nodes) {
            for (auto& [j, edgeIJ] : nodeI.edges) {
                // If we can't reach j, skip
                if (dist[i][j] == INF) continue;

                // Check all possible destinations k
                for (auto& [k, _] : nodes) {
                    long long alt = (long long)dist[i][j] + dist[j][k];
                    long long cur = dist[i][k];
                    if (alt < cur) {
                        dist[i][k] = (int)alt;
                        nextHop[i][k] = nextHop[i][j]; 
                        updated = true;
                    } else if (alt == cur && nextHop[i][j] != -1) {
                        // Tie-break: choose smaller nextHop
                        if (nextHop[i][j] < nextHop[i][k]) {
                            nextHop[i][k] = nextHop[i][j];
                            updated = true;
                        }
                    }
                }
            }
        }
    }
}

void writeForwardingTable(ofstream& outfile,
                          const map<int, map<int,int>>& dist,
                          const map<int, map<int,int>>& nextHop)
{
    // For each node i
    for (auto& [i, row] : dist) {
        outfile << "<forwarding table entries for node " << i << ">\n";
        // Gather (destination, nextHop, cost), skip unreachable
        vector<pair<int,pair<int,int>>> entries;
        for (auto& [dest, cost] : row) {
            if (cost < INF) {
                entries.push_back({dest,{nextHop.at(i).at(dest), cost}});
            }
        }
        sort(entries.begin(), entries.end(), 
            [](auto &a, auto &b) { return a.first < b.first; }
        );
        for (auto& e : entries) {
            outfile << e.first << " " << e.second.first << " " << e.second.second << "\n";
        }
    }
}

void writeMessages(ofstream& outfile,
                   const vector<string>& messages,
                   const map<int, map<int,int>>& nextHop,
                   const map<int, map<int,int>>& dist)
{
    outfile << "<message output lines>\n";
    for (auto& msgLine : messages) {
        istringstream iss(msgLine);
        int src, dst;
        string text;
        iss >> src >> dst;
        getline(iss, text);
        if (!text.empty() && text[0] == ' ') {
            text.erase(0,1);
        }

        // Attempt to reconstruct path if cost != INF
        if (dist.at(src).at(dst) == INF) {
            outfile << "from " << src << " to " << dst 
                    << " cost infinite hops unreachable message " << text << "\n";
            continue;
        }
        int current = src;
        vector<int> path;
        while (current != dst && current != -1) {
            path.push_back(current);
            current = nextHop.at(current).at(dst);
        }
        if (current == dst) {
            path.push_back(dst);
            outfile << "from " << src << " to " << dst 
                    << " cost " << dist.at(src).at(dst) 
                    << " hops ";
            for (int i = 0; i < (int)path.size(); i++) {
                outfile << path[i];
                if (i + 1 < (int)path.size()) outfile << " ";
            }
            outfile << " message " << text << "\n";
        } else {
            outfile << "from " << src << " to " << dst 
                    << " cost infinite hops unreachable message " << text << "\n";
        }
    }
}
