#include<bits/stdc++.h>
#define INFINITY 1<<30
#define WIDTH 4
#define HEIGHT 4
using namespace std;

struct node{
    int pos[2];
    float priority;
    node(int _pos[2], float _priority){
        pos[0]=_pos[0];
        pos[1]=_pos[1];
        priority=_priority;
    }
    bool operator < (const node& p) const{ return priority > p.priority; }
};

struct searchElement{
    bool isForward;
    int source[2] ={};
    int goal[2] ={};
    bool Open[WIDTH][HEIGHT];
    bool Closed[WIDTH][HEIGHT];
    float g[WIDTH][HEIGHT];
    float f[WIDTH][HEIGHT];
    float pr[WIDTH][HEIGHT];

    searchElement(bool _isForward, int _source[2], int _goal[2]){
        isForward=_isForward;
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++)Open[i][j]=false;
        if(isForward){
            source[0]=_source[0];source[1]=_source[1];
            goal[0]=_goal[0]; goal[1]=_goal[1];
            Open[source[0]][source[1]]=true;
        }else{
            source[0]=_goal[0]; source[1]=_goal[1];
            goal[0]=_source[0]; goal[1]=_source[1];
        }
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            Closed[i][j]=false;
            g[i][j]=INFINITY;
            f[i][j]=sqrt((goal[0]-i)*(goal[0]-i)+(goal[1]-j)*(goal[1]-j));//Heuristic function
        }
        //Add source to OpenQueue
        Open[source[0]][source[1]]=true;
        pr[source[0]][source[1]]=0;
        g[source[0]][source[1]]=0;
    }

    float prmin(){
        float ans = INFINITY;
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            if(Open[i][j])ans=min(pr[i][j],ans);
        }
        ///Check for implementation error
        return ans;
    }
    float fmin(){
        float ans = INFINITY;
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            if(Open[i][j])ans=min(f[i][j],ans);
        }
        ///Check for implementation error
        return ans;
    }
    float gmin(){
        float ans = INFINITY;
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            if(Open[i][j])ans=min(g[i][j],ans);
        }
        ///Check for implementation error
        return ans;
    }

    int QPopulation(){
        int population = 0;
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            if(Open[i][j])population++;
        }
        return population;
    }
    int* getN(){
        static int ans[2]={INFINITY,INFINITY};
        float minG = INFINITY;
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            if(Open[i][j]){
                if(pr[i][j]==prmin()&&g[i][j]<minG){
                    minG=g[i][j];
                    ans[0]=i;
                    ans[1]=j;
                }
            }
        }
        return ans;
    }
    float cost(int* a, int*b){
        return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]));
    }

    void printClosed(){
        for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
            if(isForward){
                if(Closed[i][j])cout<<"Forward Closed: ("<<i<<","<<j<<")"<<endl;
                if(Open[i][j])cout<<"Forward Open: ("<<i<<","<<j<<")"<<endl;
            }else{
                if(Closed[i][j])cout<<"Backward Closed: ("<<i<<","<<j<<")"<<endl;
                if(Open[i][j])cout<<"Backward Open: ("<<i<<","<<j<<")"<<endl;
            }

        }
    }
};

float MM(bool graph[WIDTH][HEIGHT][WIDTH][HEIGHT], int Source[2], int Goal[2]){
    struct searchElement F = searchElement(true, Source, Goal);
    struct searchElement B = searchElement(false, Source, Goal);
    ///Calculate epsilon Value
    float epsilon=INFINITY;
    for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++)
        for(int _i=0; _i<WIDTH; _i++)for(int _j=0; _j<HEIGHT; _j++)
            if(graph[i][j][_i][_j]){
                float dist=sqrt((i-_i)*(i-_i)+(j-_j)*(j-_j));
                epsilon=min(epsilon,dist);
            }
    ///XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
    ///Implement the actual algorithm here
    ///XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

    ///U = cost of cheapest solution found so far, initially(now) infinite
    float U = INFINITY;
    ///while(OpenF and OpenB are not null)
    while(F.QPopulation()!=0&&B.QPopulation()!=0){
        ///C = min(prminF, prminB);
        float C = min(F.prmin(),B.prmin());
        ///if(U<=max(C,fminF, fminB, gminF+gminB+epsilon)) return U;
        if(U<=max(C, max(F.fmin(), max(B.fmin(), F.gmin()+B.gmin()+epsilon)))){
            ///--------------------------------------------///
            ///Print ClosedF and ClosedB and trace the path///
            ///--------------------------------------------///
            F.printClosed();
            B.printClosed();
            return U;
        }
        ///if(C==prminF)
        if(C==F.prmin()){
            ///n = node in OpenF where prF(n)==prminF and gF(n) is minimmum;
            int* n = F.getN();
            ///move n from OpenF to ClosedF
            F.Open[n[0]][n[1]]=false;
            F.Closed[n[0]][n[1]]=true;
            ///for each child c of n
            for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
                if(graph[n[0]][n[1]][i][j]){
                    int c[2]={i,j};
                    ///if ( c ∈ OpenF ∪ ClosedF && gF(c)<=gF(n)+cost(n,c) ) continue;
                    if((F.Open[c[0]][c[1]] || F.Closed[c[0]][c[1]])&&
                       F.g[c[0]][c[1]]<=F.g[n[0]][n[1]]+F.cost(n,c)){
                        //cout<<"Continued from Region 1"<<endl;
                        ///--------///
                        ///Region 1///
                        ///--------///
                        continue;
                    }
                    ///if (c ∈ OpenF ∪ ClosedF)
                    if(F.Open[c[0]][c[1]] || F.Closed[c[0]][c[1]]){
                        ///remove c from OpenF ∪ ClosedF
                        if(F.Open[c[0]][c[1]])F.Open[c[0]][c[1]]=false;
                        if(F.Closed[c[0]][c[1]])F.Closed[c[0]][c[1]]=false;
                    }
                    ///gF(c) = gF(n) + cost(n,c);
                    F.g[c[0]][c[1]] = F.g[n[0]][n[1]]+F.cost(n,c);
                    ///add c to OpenF
                    F.Open[c[0]][c[1]] = true;
                    ///if( c belongs to OpenB )
                    if(B.Open[c[0]][c[1]]){
                        ///U = min(U, gF(c)+gb(c));
                        U = min(U, F.g[c[0]][c[1]]+B.g[c[0]][c[1]]);
                    }
                }
            }
        }else{
            ///n = node in OpenB where prB(n)==prminB and gB(n) is minimmum;
            int* n = B.getN();
            ///move n from OpenB to ClosedB
            B.Open[n[0]][n[1]]=false;
            B.Closed[n[0]][n[1]]=true;
            ///for each child c of n
            for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++){
                if(graph[n[0]][n[1]][i][j]){
                    int c[2]={i,j};
                    ///if ( c ∈ OpenB ∪ ClosedB && gB(c)<=gB(n)+cost(n,c) ) continue;
                    if((B.Open[c[0]][c[1]] || B.Closed[c[0]][c[1]])&&
                       B.g[c[0]][c[1]]<=B.g[n[0]][n[1]]+B.cost(n,c)){
                        //cout<<"Continued from Region 2"<<endl;
                        ///--------///
                        ///Region 2///
                        ///--------///
                        continue;
                    }
                    ///if (c ∈ OpenB ∪ ClosedB)
                    if(B.Open[c[0]][c[1]] || B.Closed[c[0]][c[1]]){
                        ///remove c from OpenB ∪ ClosedB
                        if(B.Open[c[0]][c[1]])B.Open[c[0]][c[1]]=false;
                        if(B.Closed[c[0]][c[1]])B.Closed[c[0]][c[1]]=false;
                    }
                    ///gB(c) = gB(n) + cost(n,c);
                    B.g[c[0]][c[1]] = B.g[n[0]][n[1]]+B.cost(n,c);
                    ///add c to OpenB
                    B.Open[c[0]][c[1]] = true;
                    ///if( c belongs to OpenF )
                    if(F.Open[c[0]][c[1]]){
                        ///U = min(U, gF(c)+gb(c));
                        U = min(U, F.g[c[0]][c[1]]+B.g[c[0]][c[1]]);
                    }
                }
            }
        }
    }

}

int main(){
    FILE *input = fopen("Graph.txt","r");
    bool adjMatrix[WIDTH][HEIGHT][WIDTH][HEIGHT];
    for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++)
        for(int _i=0; _i<WIDTH; _i++)for(int _j=0; _j<HEIGHT; _j++)
            adjMatrix[i][j][_i][_j]=false;
    int x0, y0, x1, y1;
    int source[2], goal[2];
    bool sdTaken=false;
    while(fscanf(input,"%d %d   %d %d",&x0,&y0,&x1,&y1)!=EOF){
        if(!sdTaken){
            source[0] = x0;
            source[1] = y0;
            goal[0] = x1;
            goal[1] = y1;
            sdTaken = true;
        }else{
            adjMatrix[x0][y0][x1][y1]=true;
            adjMatrix[x1][y1][x0][y0]=true;
        }
    }

    cout<<"Reasult: "<<MM(adjMatrix,source,goal)<<endl;
    /*cout<<"("<<source[0]<<","<<source[1]<<"),("<<goal[0]<<","<<goal[1]<<")"<<endl;
    for(int i=0; i<WIDTH; i++)for(int j=0; j<HEIGHT; j++)
        for(int _i=0; _i<WIDTH; _i++)for(int _j=0; _j<HEIGHT; _j++){
            if(adjMatrix[i][j][_i][_j])cout<<"("<<i<<","<<j<<")-("<<_i<<","<<_j<<")"<<endl;
        }*/
    fclose(input);
    return 0;
}
