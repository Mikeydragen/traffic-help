// Emergency Route Planner: C++17 HTTP API backend (Windows Winsock / POSIX sockets)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
using socket_t = SOCKET;
#define CLOSESOCKET closesocket
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using socket_t = int;
#define CLOSESOCKET close
#endif
using namespace std;
const double INF=numeric_limits<double>::infinity();
struct Location{string name;double x,y;};
struct Edge{int to;double w;};
struct Result{vector<int> path;double cost=INF;int expanded=0;};
struct Hospital{string name,specialty;int location;};
struct Node{Hospital h;string key;int ht=1;Node*l=nullptr,*r=nullptr;Node(Hospital v,string k):h(move(v)),key(move(k)){}};
class AVL{
 Node*root=nullptr;
 int h(Node*n){return n?n->ht:0;} int bal(Node*n){return n?h(n->l)-h(n->r):0;}
 void fix(Node*n){n->ht=1+max(h(n->l),h(n->r));}
 Node*rr(Node*y){Node*x=y->l,*t=x->r;x->r=y;y->l=t;fix(y);fix(x);return x;}
 Node*rl(Node*x){Node*y=x->r,*t=y->l;y->l=x;x->r=t;fix(x);fix(y);return y;}
 Node*put(Node*n,Hospital v,string k){if(!n)return new Node(move(v),move(k));if(k<n->key)n->l=put(n->l,move(v),move(k));else if(k>n->key)n->r=put(n->r,move(v),move(k));else return n;fix(n);int b=bal(n);if(b>1&&bal(n->l)>=0)return rr(n);if(b>1){n->l=rl(n->l);return rr(n);}if(b<-1&&bal(n->r)<=0)return rl(n);if(b<-1){n->r=rr(n->r);return rl(n);}return n;}
 void gather(Node*n,string s,vector<Hospital>&v){if(!n)return;gather(n->l,s,v);if(n->h.specialty==s)v.push_back(n->h);gather(n->r,s,v);}
 void del(Node*n){if(n){del(n->l);del(n->r);delete n;}}
public:~AVL(){del(root);}void add(Hospital v){string k=v.specialty+"|"+v.name+"|"+to_string(v.location);root=put(root,move(v),k);}vector<Hospital> find(string s){vector<Hospital>v;gather(root,s,v);return v;}
};
class Graph{
public:vector<Location>p;vector<vector<Edge>>a;vector<vector<double>>m;
 Graph(vector<Location>v):p(move(v)){int n=p.size();a.resize(n);m.assign(n,vector<double>(n,INF));for(int i=0;i<n;i++)m[i][i]=0;}
 int size(){return (int)p.size();}
 void road(int u,int v,double w){if(u<0||v<0||u>=size()||v>=size()||u==v)return;m[u][v]=m[v][u]=w;auto upd=[&](int x,int y){auto it=find_if(a[x].begin(),a[x].end(),[&](Edge e){return e.to==y;});if(it==a[x].end())a[x].push_back({y,w});else it->w=w;};upd(u,v);upd(v,u);}
 double heur(int u,int v){return .1*hypot(p[u].x-p[v].x,p[u].y-p[v].y);}
 Result search(int s,int t,string alg,string repr){int n=size();vector<double>d(n,INF);vector<int>par(n,-1);using Q=tuple<double,double,int>;priority_queue<Q,vector<Q>,greater<Q>>q;d[s]=0;q.push({alg=="astar"?heur(s,t):0,0,s});Result out;
 while(!q.empty()){auto [f,g,u]=q.top();q.pop();if(g!=d[u])continue;out.expanded++;if(u==t)break;
 auto relax=[&](int v,double w){if(!isfinite(w))return;double ng=g+w;if(ng<d[v]){d[v]=ng;par[v]=u;q.push({ng+(alg=="astar"?heur(v,t):0),ng,v});}};
 if(repr=="matrix"){for(int v=0;v<n;v++)if(v!=u)relax(v,m[u][v]);}else for(auto e:a[u])relax(e.to,e.w);}
 out.cost=d[t];if(isfinite(d[t])){for(int u=t;u!=-1;u=par[u])out.path.push_back(u);reverse(out.path.begin(),out.path.end());}return out;}
};
string esc(string s){string o;for(char c:s){if(c=='"'||c=='\\')o+='\\';if(c=='\n')o+="\\n";else o+=c;}return o;}
string jstr(string s){return "\""+esc(s)+"\"";}
string lower(string s){for(char&c:s)c=(char)tolower((unsigned char)c);return s;}
string locationsJson(Graph&g){ostringstream o;o<<"{\"locations\":[";for(int i=0;i<g.size();i++){if(i)o<<",";o<<"{\"id\":"<<i<<",\"name\":"<<jstr(g.p[i].name)<<",\"x\":"<<g.p[i].x<<",\"y\":"<<g.p[i].y<<"}";}o<<"]}";return o.str();}
string resultJson(Graph&g,Result r,string alg,string repr){ostringstream o;o<<fixed<<setprecision(3)<<"{\"algorithm\":"<<jstr(alg)<<",\"representation\":"<<jstr(repr)<<",\"travelMinutes\":";if(isfinite(r.cost))o<<r.cost;else o<<"null";o<<",\"expandedNodes\":"<<r.expanded<<",\"path\":[";for(size_t i=0;i<r.path.size();i++){if(i)o<<",";int id=r.path[i];o<<"{\"id\":"<<id<<",\"name\":"<<jstr(g.p[id].name)<<"}";}o<<"]}";return o.str();}
string param(map<string,string>&q,string k,string d=""){return q.count(k)?q[k]:d;}
map<string,string> parseQuery(string s){map<string,string>m;stringstream ss(s);string x;while(getline(ss,x,'&')){auto p=x.find('=');if(p!=string::npos)m[x.substr(0,p)]=x.substr(p+1);}return m;}
string mime(string path){if(path.size()>=4&&path.substr(path.size()-4)==".css")return "text/css";if(path.size()>=3&&path.substr(path.size()-3)==".js")return "application/javascript";return "text/html; charset=utf-8";}
string readFile(string path){ifstream f(path,ios::binary);if(!f)return "";stringstream s;s<<f.rdbuf();return s.str();}
int main(int argc,char**argv){
#ifdef _WIN32
 WSADATA wd;if(WSAStartup(MAKEWORD(2,2),&wd)!=0){cerr<<"Winsock init failed\n";return 1;}
#endif
 int port=8080;if(argc>1)port=atoi(argv[1]);
 vector<Location>loc={{"Central Junction",0,0},{"City Market",2,1},{"River Bridge",4,1},{"Tech Park",1,4},{"School Circle",3,4},{"Airport Road",6,3},{"Patient Location",5,5},{"City Hospital",7,5},{"Heart Care Hospital",8,2},{"Trauma Centre",4,7}};
 Graph g(loc);auto R=[&](int a,int b,double w){g.road(a,b,w);};R(0,1,4);R(0,3,5);R(1,2,3);R(1,3,2);R(2,5,5);R(2,4,4);R(3,4,3);R(3,6,8);R(4,6,3);R(4,9,4);R(5,7,4);R(5,8,3);R(6,7,3);R(6,9,2);R(7,8,5);R(8,9,8);
 AVL avl;avl.add({"City Hospital","general",7});avl.add({"Heart Care Hospital","cardiology",8});avl.add({"Trauma Centre","trauma",9});avl.add({"City Hospital Neurology Unit","neurology",7});
 socket_t srv=socket(AF_INET,SOCK_STREAM,0);if(srv<0){cerr<<"Socket creation failed\n";return 1;}
 int opt=1;setsockopt(srv,SOL_SOCKET,SO_REUSEADDR,(const char*)&opt,sizeof(opt));
 sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_ANY);addr.sin_port=htons((unsigned short)port);
 if(bind(srv,(sockaddr*)&addr,sizeof(addr))<0||listen(srv,16)<0){cerr<<"Could not listen on port "<<port<<"\n";CLOSESOCKET(srv);return 1;}
 cout<<"Emergency Route Planner running at http://localhost:"<<port<<"\nPress Ctrl+C to stop.\n";
 while(true){sockaddr_in client{};socklen_t len=sizeof(client);socket_t c=accept(srv,(sockaddr*)&client,&len);if(c<0)continue;
 char buf[8192];int n=(int)recv(c,buf,sizeof(buf)-1,0);if(n<=0){CLOSESOCKET(c);continue;}buf[n]=0;string req(buf);istringstream iss(req);string method,target,ver;iss>>method>>target>>ver;string path=target.substr(0,target.find('?'));map<string,string>q;if(target.find('?')!=string::npos)q=parseQuery(target.substr(target.find('?')+1));
 string body,ct="application/json; charset=utf-8";int status=200;
 if(path=="/api/locations")body=locationsJson(g);
 else if(path=="/api/route"){int s=atoi(param(q,"start","0").c_str()),t=atoi(param(q,"goal","6").c_str());string alg=lower(param(q,"algorithm","dijkstra")),rep=lower(param(q,"representation","list"));if(s<0||t<0||s>=g.size()||t>=g.size()||(alg!="dijkstra"&&alg!="astar")||(rep!="list"&&rep!="matrix")){status=400;body="{\"error\":\"Invalid route parameters\"}";}else body=resultJson(g,g.search(s,t,alg,rep),alg,rep);}
 else if(path=="/api/hospitals"){string spec=lower(param(q,"specialty","general"));auto hs=avl.find(spec);ostringstream o;o<<"{\"specialty\":"<<jstr(spec)<<",\"hospitals\":[";for(size_t i=0;i<hs.size();i++){if(i)o<<",";o<<"{\"name\":"<<jstr(hs[i].name)<<",\"specialty\":"<<jstr(hs[i].specialty)<<",\"locationId\":"<<hs[i].location<<"}";}o<<"]}";body=o.str();}
 else if(path=="/api/benchmark"){int s=atoi(param(q,"start","0").c_str()),t=atoi(param(q,"goal","6").c_str());if(s<0||t<0||s>=g.size()||t>=g.size()){status=400;body="{\"error\":\"Invalid locations\"}";}else{ostringstream o;o<<"{\"results\":[";string algs[]={"dijkstra","astar"},reps[]={"matrix","list"};bool first=true;for(auto a:algs)for(auto r:reps){auto st=chrono::steady_clock::now();auto res=g.search(s,t,a,r);auto en=chrono::steady_clock::now();double us=chrono::duration<double,micro>(en-st).count();if(!first)o<<",";first=false;o<<fixed<<setprecision(3)<<"{\"algorithm\":"<<jstr(a)<<",\"representation\":"<<jstr(r)<<",\"microseconds\":"<<us<<",\"expandedNodes\":"<<res.expanded<<",\"travelMinutes\":";if(isfinite(res.cost))o<<res.cost;else o<<"null";o<<"}";}o<<"]}";body=o.str();}}
 else if(path=="/api/road"&&method=="POST"){int a=atoi(param(q,"from","-1").c_str()),b=atoi(param(q,"to","-1").c_str());double w=atof(param(q,"minutes","-1").c_str());if(a<0||b<0||a>=g.size()||b>=g.size()||a==b||w<0){status=400;body="{\"error\":\"Invalid road parameters\"}";}else{g.road(a,b,w==0?INF:w);body="{\"ok\":true,\"message\":\"Road travel time updated in simulation\"}";}}
 else {string file=path=="/"? "index.html":path.substr(1);if(file.find("..")!=string::npos){status=400;body="Invalid path";}else{body=readFile("public/"+file);if(body.empty()){status=404;body="Not found";}else ct=mime(file);}}
 string stat=status==200?"OK":status==400?"Bad Request":"Not Found";ostringstream head;head<<"HTTP/1.1 "<<status<<" "<<stat<<"\r\nContent-Type: "<<ct<<"\r\nContent-Length: "<<body.size()<<"\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\nCache-Control: no-store\r\n\r\n";string response=head.str()+body;send(c,response.data(),(int)response.size(),0);CLOSESOCKET(c);
 }
 CLOSESOCKET(srv);
#ifdef _WIN32
 WSACleanup();
#endif
 return 0;
}
