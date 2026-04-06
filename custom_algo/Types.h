#pragma once
#include <limits>
#include <climits>


#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>


#include <mutex>
#include <atomic>
#include <thread>

#define MAX_TIMESTEP INT_MAX/2

// #include "SharedEnv.h"
#include "ActionModel.h"

namespace CustomAlgo{
	
	enum DONE{
		NOT_DONE,
		DONE
	};

	struct Int4{
		int d[4];
	};

	struct DCR{
		int loc;
		int state;

		DCR() {
			loc=-1;
			state = DONE::NOT_DONE;
		};

		DCR(int loc, int state) : loc(loc), state(state) {};
	};


	struct PIBT_C{
		int location;
		int heuristic;
		int orientation;  // 0:east, 1:south, 2:west, 3:north
		int tie_breaker;

		//constructor
		PIBT_C(int location, int heuristic, int orientation, int tie_breaker):
			location(location), heuristic(heuristic), orientation(orientation), tie_breaker(tie_breaker) {};
	};

	struct HNode
		{
			int label;		
			int location;			// Current Location
			int direction;			// Orientation
			int value;				// Cost
			int other;

			//Modifikasi untuk InterHT
			int idx;

			unsigned int priority;
			unsigned int get_priority() const { return priority; }
    		void set_priority(unsigned int p) { priority = p; }


			HNode() = default;
			HNode(const HNode& node) = default;
			HNode& operator=(const HNode& other) = default;

			HNode(int location,int direction, int value) : location(location), direction(direction), value(value) {}
			HNode(int location, int direction) : location(location), direction(direction) {}
			
			// Modifikasi u/ InterHT
			// HNode(int value, int idx) : value(value), idx(idx) {}
			static HNode createForInterHT(int value,int idx) {
				HNode n;
				n.value = value;
				n.idx = idx;
				return n;
			}

			bool operator<(const HNode& other) const {
				return value < other.value;
			}

			bool operator>(const HNode& other) const {
				return value > other.value;
			}

			// the following is used to compare nodes in the OPEN list
			struct compare_node
			{
				bool operator()(const HNode& n1, const HNode& n2) const
				{
					return n1.value > n2.value;
				}
			};  // used by OPEN (open) to compare nodes (top of the open has min f-val, and then highest g-val)
		};

	

	

	struct Highways{
		std::map<std::pair<int,int>,std::pair<int,int>> e_hw;
        std::map<std::pair<int,int>,std::pair<int,int>> r_e_hw;

		Highways() = default;
	};
	
	struct Entrances{
		int loc_a;
		int c_a;
		int neigh;
		int c_b;

		Entrances() : c_a(-1), c_b(-1) {};
	};

	struct HPA_H {

		struct AbstractGraph{
			std::vector<int> gates;	//Daftar seluruh gates secara gabungan
			std::vector<int> gate_index; // Memetakan lokasi global gate dengan sebuah index

			std::map<std::pair<int,int>, int> intra;
			std::map<std::pair<int,int>, int> inter;
			std::map<int,std::vector<int>> neighbors;

			AbstractGraph() = default;
		};
		AbstractGraph AG;

		//Cluster, Destination Local, Source Local, Orientation.HT antara gates (dalam cluster) 
		std::vector<std::map<int, std::vector<std::array<int,4>>>> IntraHT;
		std::vector<std::vector<int>> InterHT; //HT antar gates (diluar cluster)
		
		std::vector<std::vector<int>> Gates;	//Menyimpan seluruh gate pada masing-masing cluster

		

		std::vector<Entrances> Ents;
		std::vector<std::vector<int>> local_to_global;
		std::vector<int> global_to_local;
		std::vector<int> voronoi_map;
		Highways hw;

		std::map<int, std::vector<int>> inter_cache;

		HPA_H() = default;
	};

	struct PlannerState{
		int w;	//Planning horizon / window size
		int h;  //Replan Period

		//GCM[loc][orient] untuk heat mapnya
		std::vector<std::array<float,4>> gcm;
		
		//[loc][orient] untuk
		std::vector<std::array<int,4>> wait_map;
	
		int gcm_freq = 5; // update wait_map setiap F frequency

		//Struct untuk simpen nilai wait_peak setiap lokasi
		struct PeakInfo{
			float val = 0.0f;
			int timestep = 0;
		};
		std::vector<PeakInfo> w_peak;

		//Baseline weight = 1, asumsi lokasi pasti pernah sekali dilewatin / akan dilewatin
		float w_baseline = 1.0f;

		// Cooling constant, fisika punya yang namanya Stefan's law untuk radiation cooling tapi kurang bisa dipake angkanya
		/*
			k = k[v] = k_base |Neigh[v]|/delta(G)
			k = k[v] = k_base * {1 sampai 1/4}
			k = 0.1 sampai 0.025

			exp(-0.1t) sampai exp(-0.025t) = 0.9,0.97 (t=1), 0.6,0.88 (t=5)
			Untuk sementara, ditaro di 0.1 atau 10%
		*/
		float k_base = 0.1f; 

		int max_degree = 4;

		//Plan dari window terakhir , plans[agt_id][step] = loc;
		std::vector<std::vector<int>> current_plans;

		// //Disabled Agents (lokasi dead-end). Dari paper, dan github. Jika exit dan entry hanya ada satu. Mencegah deadlock
		// std::unordered_set<int> disabled;


		//Data dari PIBT
		std::vector<double> priorities;
		std::vector<double> priorities_base;
		std::vector<DCR> decided;
	};

	enum class DestroyHeuristic { AGENT_BASED, RANDOM, MAP_BASED, COUNT };
	
	struct LNS{

		std::mutex mtx;
		std::vector<std::vector<int>> P_min;
		int P_min_soc = INTERVAL_MAX;

		std::array<float, (int)DestroyHeuristic::COUNT> weights;
		
		std::atomic<bool> stop_flag;

		LNS() { 
			weights.fill(1.0f); 
		};
	};
	typedef std::vector<std::vector<int>> Neighbors;

}







