#include <cstdio>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <algorithm>
#include <limits>

using namespace std;

struct Edge {
    int to;
    int cost;
};

struct Node {
    int id;
    map<int, Edge> edges;
};

void parseTopologyFile(const string& filename, map<int, Node>& nodes);
void parseMessageFile(const string& filename, vector<string>& messages);
void parseChangesFile(const string& filename, vector<tuple<int, int, int>>& changes);
void writeForwardingTable(ofstream& outfile, const map<int, Node>& nodes, map<int, map<int, int>>& routeNext, map<int, map<int, int>>& routeCost);
void writeMessages(ofstream& outfile, const vector<string>& messages, const map<int, map<int, int>>& routeNext, const map<int, map<int, int>>& routeCost);

void writeForwardingTable(ofstream& outfile, const map<int, Node>& nodes, map<int, map<int, int>>& routeNext, map<int, map<int, int>>& routeCost) {
    for (const auto& [id, node] : nodes) {
        // Compute shortest paths using Dijkstra's algorithm
        map<int, int> dist;
        map<int, int> nextHop;
        map<int, int> prevHop;
        set<pair<int, int>> priority_queue;

        for (const auto& [nid, _] : nodes) {
            dist[nid] = numeric_limits<int>::max();
        }
        dist[id] = 0;
        nextHop[id] = id; // The next hop for the node itself is the node itself
        prevHop[id] = id;
        priority_queue.insert({0, id});

        while (!priority_queue.empty()) {
            int u = priority_queue.begin()->second;
            priority_queue.erase(priority_queue.begin());

            for (const auto& [v, edge] : nodes.at(u).edges) {
                int alt = dist[u] + edge.cost;
                if (alt < dist[v] || (alt == dist[v] && u < prevHop[v])) {
                    priority_queue.erase({dist[v], v});
                    dist[v] = alt;
                    nextHop[v] = (u == id) ? v : nextHop[u];
                    prevHop[v] = u;
                    priority_queue.insert({dist[v], v});
                }
            }
        }

        // Store the computed routes
        routeNext[id] = nextHop;
        routeCost[id] = dist;

        // Write forwarding table for node id
        vector<pair<int, pair<int, int>>> tableEntries;
        for (const auto& [dest, cost] : dist) {
            if (cost < numeric_limits<int>::max()) {
                tableEntries.push_back({dest, {nextHop[dest], cost}});
            }
        }
        sort(tableEntries.begin(), tableEntries.end());

        // Skip printing if the table contains only the node's entry for itself
        //if (tableEntries.size() == 1 && tableEntries[0].first == id) {
        //    continue;
       // }

        //outfile << "<forwarding table entries for node " << id << ">\n";
        for (const auto& entry : tableEntries) {
            outfile << entry.first << " " << entry.second.first << " " << entry.second.second << "\n";
        }
        outfile << "\n"; // Add blank line between tables
    }
}

void parseTopologyFile(const string& filename, map<int, Node>& nodes) {
    ifstream infile(filename);
    string line;
    while (getline(infile, line)) {
        if (line.empty()) continue; // Skip blank lines
        istringstream iss(line);
        int from, to, cost;
        if (iss >> from >> to >> cost) {
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
        if (line.empty()) continue; // Skip blank lines
        messages.push_back(line);
    }
}

void parseChangesFile(const string& filename, vector<tuple<int, int, int>>& changes) {
    ifstream infile(filename);
    string line;
    while (getline(infile, line)) {
        if (line.empty()) continue; // Skip blank lines
        istringstream iss(line);
        int from, to, cost;
        if (iss >> from >> to >> cost) {
            changes.emplace_back(from, to, cost);
        }
    }
}

void writeMessages(ofstream& outfile, const vector<string>& messages, const map<int, map<int, int>>& routeNext, const map<int, map<int, int>>& routeCost) {
    // outfile << "<message output lines>\n";
    for (const auto& message : messages) {
        istringstream iss(message);
        int src, dst;
        string msg;
        iss >> src >> dst;
        getline(iss, msg);
        msg = msg.substr(1); // Remove leading space

        // Find path from src to dst
        vector<int> path;
        int current = src;
        bool pathExists = true;

        while (current != dst) {
            path.push_back(current);
            if (routeNext.at(current).find(dst) == routeNext.at(current).end()) {
                pathExists = false;
                break;
            }
            current = routeNext.at(current).at(dst);
        }
        if (pathExists) {
            // Write message with path excluding the destination node
            outfile << "from " << src << " to " << dst << " cost " << routeCost.at(src).at(dst) << " hops ";
            for (size_t i = 0; i < path.size(); ++i) {
                outfile << path[i];
                if (i < path.size() - 1) {
                    outfile << " ";
                }
            }
            outfile << " message " << msg << "\n";
        } else {
            outfile << "from " << src << " to " << dst << " cost infinite hops unreachable message " << msg << "\n";
        }
        
    }
    outfile << "\n"; // Add blank line between tables
}

int main(int argc, char** argv) {
    if (argc != 4) {
        printf("Usage: ./linkstate topofile messagefile changesfile\n");
        return -1;
    }

    map<int, Node> nodes;
    vector<string> messages;
    vector<tuple<int, int, int>> changes;
    map<int, map<int, int>> routeNext;
    map<int, map<int, int>> routeCost;

    parseTopologyFile(argv[1], nodes);
    parseMessageFile(argv[2], messages);
    parseChangesFile(argv[3], changes);

    ofstream outfile("output.txt");

    // Initial state
    writeForwardingTable(outfile, nodes, routeNext, routeCost);
    writeMessages(outfile, messages, routeNext, routeCost);

    // Apply changes and write state after each change
    for (const auto& [from, to, cost] : changes) {
        if (cost == -999) {
            nodes[from].edges.erase(to);
            nodes[to].edges.erase(from);
        } else {
            nodes[from].edges[to] = {to, cost};
            nodes[to].edges[from] = {from, cost};
        }
        // outfile << "----- At this point, change is applied\n";
        writeForwardingTable(outfile, nodes, routeNext, routeCost);
        writeMessages(outfile, messages, routeNext, routeCost);
    }

    outfile.close();
    return 0;
}