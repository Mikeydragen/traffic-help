/*
 Optimized Route Planner for Emergency Vehicles
 C++17 educational DSA project.
 NOTE: Uses simulated city/traffic data; not for real emergency dispatch.
*/
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <vector>
using namespace std;

const double INF = numeric_limits<double>::infinity();

struct Location {
    string name;
    double x, y; // illustrative map coordinates, not real GPS
};
struct Edge { int to; double minutes; };
struct PathResult {
    vector<int> path;
    double minutes = INF;
    long long expanded = 0;
};

class Graph {
public:
    vector<Location> nodes;
    vector<vector<Edge>> adj;
    vector<vector<double>> matrix;

    explicit Graph(vector<Location> locations) : nodes(move(locations)) {
        int n = (int)nodes.size();
        adj.resize(n);
        matrix.assign(n, vector<double>(n, INF));
        for (int i=0;i<n;i++) matrix[i][i]=0;
    }
    int size() const { return (int)nodes.size(); }

    void addRoad(int a, int b, double minutes) {
        if (a<0 || b<0 || a>=size() || b>=size() || minutes<0) return;
        // Store the same road in both directions for this sample city.
        updateEdge(a,b,minutes);
        updateEdge(b,a,minutes);
    }
    void updateEdge(int a, int b, double minutes) {
        matrix[a][b] = minutes;
        auto it = find_if(adj[a].begin(), adj[a].end(),
                          [b](const Edge& e){ return e.to==b; });
        if (it == adj[a].end()) adj[a].push_back({b,minutes});
        else it->minutes=minutes;
    }
    void setRoadTime(int a, int b, double minutes) {
        if (a<0 || b<0 || a>=size() || b>=size() || a==b || minutes<0) return;
        updateEdge(a,b,minutes);
        updateEdge(b,a,minutes);
    }
    double heuristic(int a, int b) const {
        // Conservative scale keeps the sample heuristic small/admissible.
        return 0.1 * hypot(nodes[a].x-nodes[b].x, nodes[a].y-nodes[b].y);
    }

    PathResult dijkstra(int start, int goal, bool useMatrix) const {
        int n=size();
        vector<double> dist(n,INF);
        vector<int> parent(n,-1);
        using Item=pair<double,int>;
        priority_queue<Item,vector<Item>,greater<Item>> pq;
        dist[start]=0; pq.push({0,start});
        PathResult result;
        while(!pq.empty()) {
            auto [d,u]=pq.top(); pq.pop();
            if(d!=dist[u]) continue;
            result.expanded++;
            if(u==goal) break;
            if(useMatrix) {
                for(int v=0;v<n;v++) {
                    if(v==u || !isfinite(matrix[u][v])) continue;
                    double nd=d+matrix[u][v];
                    if(nd<dist[v]) { dist[v]=nd; parent[v]=u; pq.push({nd,v}); }
                }
            } else {
                for(const auto& e:adj[u]) {
                    double nd=d+e.minutes;
                    if(nd<dist[e.to]) { dist[e.to]=nd; parent[e.to]=u; pq.push({nd,e.to}); }
                }
            }
        }
        result.minutes=dist[goal];
        if(isfinite(dist[goal])) {
            for(int cur=goal;cur!=-1;cur=parent[cur]) result.path.push_back(cur);
            reverse(result.path.begin(),result.path.end());
        }
        return result;
    }

    PathResult aStar(int start, int goal, bool useMatrix) const {
        int n=size();
        vector<double> g(n,INF);
        vector<int> parent(n,-1);
        using Item=tuple<double,double,int>; // f, g, node
        priority_queue<Item,vector<Item>,greater<Item>> pq;
        g[start]=0; pq.push({heuristic(start,goal),0,start});
        PathResult result;
        while(!pq.empty()) {
            auto [f,curG,u]=pq.top(); pq.pop();
            if(curG!=g[u]) continue;
            result.expanded++;
            if(u==goal) break;
            auto relax=[&](int v,double w) {
                double ng=curG+w;
                if(ng<g[v]) {
                    g[v]=ng; parent[v]=u;
                    pq.push({ng+heuristic(v,goal),ng,v});
                }
            };
            if(useMatrix) {
                for(int v=0;v<n;v++)
                    if(v!=u && isfinite(matrix[u][v])) relax(v,matrix[u][v]);
            } else {
                for(const auto& e:adj[u]) relax(e.to,e.minutes);
            }
        }
        result.minutes=g[goal];
        if(isfinite(g[goal])) {
            for(int cur=goal;cur!=-1;cur=parent[cur]) result.path.push_back(cur);
            reverse(result.path.begin(),result.path.end());
        }
        return result;
    }
};

struct Hospital {
    string name, specialty;
    int location;
};
struct AVLNode {
    Hospital hospital;
    string key;
    int height=1;
    AVLNode *left=nullptr,*right=nullptr;
    AVLNode(Hospital h,string k):hospital(move(h)),key(move(k)){}
};
class HospitalAVL {
    AVLNode* root=nullptr;
    int height(AVLNode* n){return n?n->height:0;}
    int balance(AVLNode* n){return n?height(n->left)-height(n->right):0;}
    void fix(AVLNode* n){n->height=1+max(height(n->left),height(n->right));}
    AVLNode* rotateRight(AVLNode* y){
        AVLNode* x=y->left; AVLNode* t=x->right;
        x->right=y; y->left=t; fix(y); fix(x); return x;
    }
    AVLNode* rotateLeft(AVLNode* x){
        AVLNode* y=x->right; AVLNode* t=y->left;
        y->left=x; x->right=t; fix(x); fix(y); return y;
    }
    AVLNode* insert(AVLNode* n,Hospital h,string key){
        if(!n) return new AVLNode(move(h),move(key));
        if(key<n->key) n->left=insert(n->left,move(h),move(key));
        else if(key>n->key) n->right=insert(n->right,move(h),move(key));
        else { // unique composite key; append a suffix if caller supplies duplicate
            n->right=insert(n->right,move(h),move(key)+"~");
        }
        fix(n); int b=balance(n);
        if(b>1 && balance(n->left)>=0) return rotateRight(n);
        if(b>1 && balance(n->left)<0){n->left=rotateLeft(n->left);return rotateRight(n);}
        if(b<-1 && balance(n->right)<=0) return rotateLeft(n);
        if(b<-1 && balance(n->right)>0){n->right=rotateRight(n->right);return rotateLeft(n);}
        return n;
    }
    void collect(AVLNode* n,const string& specialty,vector<Hospital>& out) const {
        if(!n) return;
        collect(n->left,specialty,out);
        if(n->hospital.specialty==specialty) out.push_back(n->hospital);
        collect(n->right,specialty,out);
    }
    void destroy(AVLNode* n){if(!n)return;destroy(n->left);destroy(n->right);delete n;}
public:
    ~HospitalAVL(){destroy(root);}
    void add(Hospital h){
        string key=h.specialty+"|"+h.name+"|"+to_string(h.location);
        root=insert(root,move(h),key);
    }
    vector<Hospital> bySpecialty(const string& specialty) const {
        vector<Hospital> out; collect(root,specialty,out); return out;
    }
};

string lower(string s){for(char& c:s)c=(char)tolower((unsigned char)c);return s;}
int readInt(const string& prompt,int lo,int hi){
    while(true){cout<<prompt;string s;getline(cin,s);
        try{size_t p;int v=stoi(s,&p);if(p==s.size()&&v>=lo&&v<=hi)return v;}catch(...){}
        cout<<"Please enter a number from "<<lo<<" to "<<hi<<".\n";
    }
}
void printPath(const Graph& g,const PathResult& r){
    if(!isfinite(r.minutes)||r.path.empty()){cout<<"No route found.\n";return;}
    cout<<fixed<<setprecision(2)<<"Estimated travel time: "<<r.minutes<<" minutes\nRoute: ";
    for(size_t i=0;i<r.path.size();i++){if(i)cout<<" -> ";cout<<g.nodes[r.path[i]].name;}
    cout<<"\nLocations explored: "<<r.expanded<<"\n";
}
void benchmark(const Graph& g,int s,int t){
    cout<<"\nPerformance comparison (same simulated network/query)\n";
    cout<<left<<setw(23)<<"Algorithm / storage"<<setw(18)<<"Time (microsec)"<<setw(16)<<"Nodes explored"<<"Route minutes\n";
    auto run=[&](string label,auto fn){
        auto begin=chrono::steady_clock::now(); PathResult r=fn();
        auto end=chrono::steady_clock::now();
        double us=chrono::duration<double,micro>(end-begin).count();
        cout<<left<<setw(23)<<label<<setw(18)<<fixed<<setprecision(2)<<us<<setw(16)<<r.expanded;
        if(isfinite(r.minutes))cout<<r.minutes;else cout<<"unreachable";
        cout<<"\n";
    };
    run("Dijkstra + matrix",[&]{return g.dijkstra(s,t,true);});
    run("Dijkstra + list",[&]{return g.dijkstra(s,t,false);});
    run("A* + matrix",[&]{return g.aStar(s,t,true);});
    run("A* + list",[&]{return g.aStar(s,t,false);});
    cout<<"Timing is illustrative and can vary by machine; repeat tests on larger graphs for conclusions.\n";
}
int main(){
    ios::sync_with_stdio(false);cin.tie(nullptr);
    vector<Location> locations={
        {"Central Junction",0,0},{"City Market",2,1},{"River Bridge",4,1},
        {"Tech Park",1,4},{"School Circle",3,4},{"Airport Road",6,3},
        {"Patient Location",5,5},{"City Hospital",7,5},{"Heart Care Hospital",8,2},
        {"Trauma Centre",4,7}
    };
    Graph graph(locations);
    auto road=[&](int a,int b,double m){graph.addRoad(a,b,m);};
    road(0,1,4);road(0,3,5);road(1,2,3);road(1,3,2);road(2,5,5);
    road(2,4,4);road(3,4,3);road(3,6,8);road(4,6,3);road(4,9,4);
    road(5,7,4);road(5,8,3);road(6,7,3);road(6,9,2);road(7,8,5);road(8,9,8);

    HospitalAVL hospitals;
    hospitals.add({"City Hospital","general",7});
    hospitals.add({"Heart Care Hospital","cardiology",8});
    hospitals.add({"Trauma Centre","trauma",9});
    hospitals.add({"City Hospital Neurology Unit","neurology",7});

    while(true){
        cout<<"\n========== EMERGENCY ROUTE PLANNER (C++17) ==========\n"
            <<"1. Find route using Dijkstra (adjacency list)\n"
            <<"2. Find route using A* (adjacency list)\n"
            <<"3. Find suitable hospitals by specialty (AVL Tree)\n"
            <<"4. Compare all four algorithm/storage combinations\n"
            <<"5. Simulate traffic change / road closure\n"
            <<"6. Display city locations\n"
            <<"0. Exit\n";
        int choice=readInt("Choose option: ",0,6);
        if(choice==0){cout<<"Thank you. This was a simulated demonstration.\n";break;}
        if(choice==6){
            for(int i=0;i<graph.size();i++)cout<<i<<" - "<<graph.nodes[i].name<<"\n";
            continue;
        }
        if(choice==1||choice==2||choice==4){
            cout<<"Enter location IDs shown in the location list.\n";
            int s=readInt("Start location ID: ",0,graph.size()-1);
            int t=readInt("Destination location ID: ",0,graph.size()-1);
            if(choice==1){cout<<"\nDijkstra result\n";printPath(graph,graph.dijkstra(s,t,false));}
            else if(choice==2){cout<<"\nA* result\n";printPath(graph,graph.aStar(s,t,false));}
            else benchmark(graph,s,t);
        } else if(choice==3){
            cout<<"Available specialties: general, cardiology, trauma, neurology\n";
            cout<<"Enter specialty: ";string specialty;getline(cin,specialty);specialty=lower(specialty);
            auto found=hospitals.bySpecialty(specialty);
            if(found.empty())cout<<"No hospital found for that specialty.\n";
            else{
                cout<<"Matching hospitals:\n";
                for(const auto& h:found){
                    cout<<"- "<<h.name<<" (location ID "<<h.location<<")\n";
                }
                int patient=readInt("Patient location ID: ",0,graph.size()-1);
                double best=INF; const Hospital* selected=nullptr; PathResult bestPath;
                for(const auto& h:found){
                    PathResult r=graph.dijkstra(patient,h.location,false);
                    if(r.minutes<best){best=r.minutes;selected=&h;bestPath=r;}
                }
                if(selected){cout<<"\nNearest reachable matching hospital: "<<selected->name<<"\n";printPath(graph,bestPath);}
                else cout<<"No reachable matching hospital.\n";
            }
        } else if(choice==5){
            cout<<"Locations:\n";for(int i=0;i<graph.size();i++)cout<<i<<" - "<<graph.nodes[i].name<<"\n";
            int a=readInt("First endpoint ID: ",0,graph.size()-1);
            int b=readInt("Second endpoint ID: ",0,graph.size()-1);
            if(a==b){cout<<"Choose two different endpoints.\n";continue;}
            cout<<"Enter new travel time in minutes (0 closes this road): ";
            string s;getline(cin,s);double m;
            try{m=stod(s);if(m<0)throw invalid_argument("negative");}
            catch(...){cout<<"Invalid travel time.\n";continue;}
            if(m==0){graph.setRoadTime(a,b,INF);cout<<"Road closed in the simulation. (A closed road is represented by infinity.)\n";}
            else {graph.setRoadTime(a,b,m);cout<<"Road travel time updated.\n";}
            // INF is supported as an unavailable edge in both representations.
        }
    }
    return 0;
}
