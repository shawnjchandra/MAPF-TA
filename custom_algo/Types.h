#pragma once
#include <limits.h>


#include <vector>
#include <iostream>
#include <deque>
#include <regex>
#include <fstream>
#include <cassert>
#include <unordered_set>


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
			// the following is used to compare nodes in the OPEN list
			struct compare_node
			{
				bool operator()(const HNode& n1, const HNode& n2) const
				{
					return n1.value > n2.value;
				}
			};  // used by OPEN (open) to compare nodes (top of the open has min f-val, and then highest g-val)
		};

	struct AbstractGraph{
		std::vector<int> gates;
		std::map<std::pair<int,int>, int> intra;
		std::map<std::pair<int,int>, int> inter;
		std::map<int,std::vector<int>> neighbors;

		AbstractGraph() = default;
	};

	struct Entrances{
		int loc_a;
		int c_a;
		int neigh;
		int c_b;

		Entrances() : c_a(-1), c_b(-1) {};
	};

	struct Highways{
		std::map<std::pair<int,int>,std::pair<int,int>> e_hw;
        std::map<std::pair<int,int>,std::pair<int,int>> r_e_hw;

		Highways() = default;
	};

	struct HPA_H {
		AbstractGraph AG;
		std::vector<std::vector<std::vector<std::array<int,4>>>> IntraHT; //Cluster, Destination Local, Source Local, Orientation
		std::vector<std::vector<int>> InterHT;
		std::vector<std::vector<int>> Gates;
		std::vector<Entrances> Ents;
		std::vector<std::vector<int>> local_to_global;
		std::vector<int> global_to_local;
		std::vector<int> gate_index;
		std::vector<int> voronoi_map;
		Highways hw;

		HPA_H() = default;
	};


	struct Degree_Map {
		std::vector<int> degree_map;

		Degree_Map() = default;
	};

	typedef std::vector<std::vector<int>> Neighbors;

}







