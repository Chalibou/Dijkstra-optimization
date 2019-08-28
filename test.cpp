// A C++ program for Dijkstra's single source shortest path algorithm. 
// The program is for adjacency matrix representation of the graph 
//https://www.geeksforgeeks.org/c-program-for-dijkstras-shortest-path-algorithm-greedy-algo-7/


#include <limits.h> 
#include <stdio.h> 
#include <string>
#include <chrono>  // for high_resolution_clock
#include <stdlib.h> // for random function 
#include <stdexcept> //for errors

using namespace std;

//Debug and usefull fonctions
void printIntArray(int array[], int size, string name){
	std::printf("## Print of array %s \n ------------- \n", name.c_str()); 
	for (int i = 0; i < size; i++){
		std::printf("%d\n", array[i]); 
	}
	std::printf("------------- \n");
};
void printStringArray(string array[], int size, string name){
	std::printf("Print of array %s \n", name.c_str()); 
	for (int i = 0; i < size; i++){
		std::printf("%s\n", array[i].c_str()); 
	}
};


// Record start time
auto start = std::chrono::high_resolution_clock::now();

//Convert number to letter
string convertToLetter(int number){
	//Argument errors handling
	if (number>130) {
        return "OUT OF RANGE";
    }
	string alphabet[130] = {"A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X","Y","Z","AA","AB","AC","AD","AE","AF","AG","AH","AI","AJ","AK","AL","AM","AN","AO","AP","AQ","AR","AS","AT","AU","AV","AW","AX","AY","AZ","BA","BB","BC","BD","BE","BF","BG","BH","BI","BJ","BK","BL","BM","BN","BO","BP","BQ","BR","BS","BT","BU","BV","BW","BX","BY","BZ","CA","CB","CC","CD","CE","CF","CG","CH","CI","CJ","CK","CL","CM","CN","CO","CP","CQ","CR","CS","CT","CU","CV","CW","CX","CY","CZ","DA","DB","DC","DD","DE","DF","DG","DH","DI","DJ","DK","DL","DM","DN","DO","DP","DQ","DR","DS","DT","DU","DV","DW","DX","DY","DZ"};
	string response = alphabet[number];
	return response;
}


// A utility function to find the vertex with minimum distance value, from 
// the set of vertices not yet included in shortest path tree 
int minDistance(int dist[], bool sptSet[],int size) 
{ 
	// Initialize min value 
	int min = INT_MAX, min_index; 

	for (int v = 0; v < size; v++) 
		if (sptSet[v] == false && dist[v] <= min) 
			min = dist[v], min_index = v; 

	return min_index; 
} 

//build back entire path to "source" thanks to the "origin" array obtained from Dijkstra
void buildPath(int origin[],string path[],int source,int size){
		
	//"Goal" is the objective letter of the pathfinding 
	string goal = "-"+convertToLetter(source);

	for (int i=0;i<size;i++){
		//"Origin" array may have 2 types of values : -1 == no path found | >=0 == you have to pass through another node before reaching "source", this info is in "origin"
		string completePath="";
		completePath += "["+convertToLetter(i);
		int d = origin[i];
		if(d == -1){path[i]="NO PATH";continue;} //No path found
		while(d >= 0){ //"Origin" gives the next node to reach, you go on until you arive at 0
			completePath += "-"+convertToLetter(d);
			d = origin[d];
		}
		path[i]=completePath + "]";
	}
}
	

// A utility function to print the constructed distance array 
void printSolution(int dist[],int origin[],string path[],int src, int size){ 
	std::printf("Vertex Distance from Source : %s \n",convertToLetter(src).c_str()); 
	for (int i = 0; i < size; i++){
		std::printf("%s longueur : %d, chemin : %s\n", convertToLetter(i).c_str(), dist[i], path[i].c_str()); 
	}
} 

// Function that implements Dijkstra's single source shortest path algorithm 
// for a graph represented using adjacency matrix representation 
void dijkstra(int **graph, int src,int size) 
{ 
	int *dist = new int[size]; // An output array. dist[i] will hold the shortest 
	// distance from src to i 

	int *origin = new int[size]; // An output array. path[i] will hold the origin of the shortes path for each node
	
	//[Problem]
	//This method actually don't put all values to -1 so we do it manually
	for(int i=0;i<size;i++){
		origin[i]=-1;
	}

	string *path = new string[size]; // An output array. path[i] will hold the origin of the shortes path for each node

	bool *sptSet = new bool[size]; // sptSet[i] will be true if vertex i is included in shortest 
	// path tree or shortest distance from src to i is finalized 

	// Initialize all distances as INFINITE and stpSet[] as false 
	for (int i = 0; i < size; i++) 
		dist[i] = INT_MAX, sptSet[i] = false; 

	// Distance of source vertex from itself is always 0 
	dist[src] = 0; 

	// Find shortest path for all vertices 
	for (int count = 0; count < size - 1; count++) { 
		// Pick the minimum distance vertex from the set of vertices not 
		// yet processed. u is always equal to src in the first iteration. 
		int u = minDistance(dist, sptSet,size); 

		// Mark the picked vertex as processed 
		sptSet[u] = true; 

		// Update dist value of the adjacent vertices of the picked vertex. 
		for (int v = 0; v < size; v++){

			// Update dist[v] only if is not in sptSet, there is an edge from 
			// u to v, and total weight of path from src to v through u is 
			// smaller than current value of dist[v] 
			if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]){
				dist[v] = dist[u] + graph[u][v];
				origin[v] = u; //Register origin of shorter path
			} 
		} 	
	}

	//build back entire path to 0
	buildPath(origin,path,src,size);
	
	// print the constructed distance array 
	printSolution(dist,origin,path,src,size); 

} 

//Generate a random map given node count and link per node count, links are generated randomly such as weight for each node
int** genMap(int size, int linkQty, int maxWeight){
	//Argument errors handling
	if ( size < 0 || size < linkQty ) {
        throw std::invalid_argument( "Invalid arguments" );
    }
	int val = 0;
	float decision;
	int count = 0;
	float sensivity = 0.25;//Control repartition within the matrix, good value is 0.25
	//Initialize random seed
	srand (time(NULL));

	//Create result array with pointer of pointer (so the 2d array can have dynamic size)
	int** result = new int*[size];
	for (int i=0; i<size; i++){
		//Filling the array with arrays
		result[i]= new int[size];
		//we have to produce symetrical array so j is limited to i
		for (int j=0; j<=i; j++){
			if(i==j){result[i][j] =0;continue;} //Array symetry with 0 in main diagonal
			//Decision for randomizing [0 1] position of the values in the array 
			decision = rand()/(float)RAND_MAX;
			//If decision is ok and we still have not reached the link quantity per node value then we put some value or 0
			if ((decision <sensivity) & (count<(linkQty/2))){
				result[i][j] = rand() % maxWeight + 1;
				count++;
			}else{
				result[i][j] = 0;
			}
			result[j][i] = result[i][j];
		}
		count=0;
	}
/* 
	//Print matrix for debugging
	std::printf("Generated symetrical 2d array\n"); 
	for (int i = 0; i < size; i++){
		for (int j = 0; j < size; j++){
			std::printf("%d\t", result[i][j]); 
		}
		std::printf("\n"); 
	}
*/
	return result;
}

// driver program to test above function 
int main() 
{ 
	int SIZE=10;
	int LINK_COUNT=5;
	int MAX_WEIGHT=15;
	//Generate a symetrical array with dynamic size and specific distribution as shown in the example below :
	int** genGraph = genMap(SIZE,LINK_COUNT,MAX_WEIGHT);
	int START = 8;

	/* 
	//Let us create the example graph discussed above 
	int graph[V][V] = { 
            { 0, 4, 0, 0, 0, 0, 0, 8, 0 }, 
			{ 4, 0, 8, 0, 0, 0, 0, 11, 0 }, 
			{ 0, 8, 0, 7, 0, 4, 0, 0, 2 }, 
			{ 0, 0, 7, 0, 9, 14, 0, 0, 0 }, 
			{ 0, 0, 0, 9, 0, 10, 0, 0, 0 }, 
			{ 0, 0, 4, 14, 10, 0, 2, 0, 0 }, 
			{ 0, 0, 0, 0, 0, 2, 0, 1, 6 }, 
			{ 8, 11, 0, 0, 0, 0, 1, 0, 7 }, 
			{ 0, 0, 2, 0, 0, 0, 6, 7, 0 } }; 
	*/

	dijkstra(genGraph, START, SIZE); 

	//Record finish time and get the elapsed time in seconds
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::printf("Time of execution :  %f seconds", elapsed.count());

  return 0; 
} 