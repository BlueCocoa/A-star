//
//  Astar.hpp
//  A* Search Algorithm
//
//  Created by BlueCocoa on 2016/8/28.
//  Copyright © 2016 BlueCocoa. All rights reserved.
//

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <stdlib.h>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <vector>

/**
 *  @brief A* Search Algorithm
 *
 *  @param nodes        Number of nodes
 *  @param graph        2-dimension ajacent matrix
 *  @param initial      Starting node
 *  @param goal         Target node
 *  @param estimated    Heuristic function
 *
 *  @note  If estimated function is not provided, A* search is equivalent to uniformed-cost search
 */
template <typename Weight>
std::vector<size_t> Astar(size_t nodes, const Weight ** graph, const size_t initial, const size_t goal, const std::function<Weight(const size_t current)>& estimated = [](const size_t current) -> Weight { return 0; }) {
    // return an empty std::vector on invaild input
    if (nodes == 0 || graph == nullptr) return std::vector<size_t>();
    
    // return <initial, goal> if initial equals to goal
    if (initial == goal) return std::vector<size_t>(initial, goal);
    
    // state: current node, via, g(n), h(n)
    // g(n):  cost so far to reach n
    // h(n):  estimated cost to goal from n
    using state = std::tuple<size_t, size_t, Weight, Weight>;
    
    // compare function for std::priority_queue
    auto cmp = [](const state& a, const state& b) {
        // (g(a) + h(a)) > (g(b) + h(b))
        // a.k.a sort by estimated total cost from initial node to goal through n
        return ((std::get<2>(a) + std::get<3>(a)) > (std::get<2>(b) + std::get<3>(b)));
    };
    
    // frontier, increasing order
    std::priority_queue<state, std::vector<state>, decltype(cmp)> frontier(cmp);
    
    // visited node
    std::set<size_t> visited;
    
    // froniter set
    std::set<size_t> froniter_set;
    
    // record trace
    size_t * visit = (size_t *)malloc(sizeof(size_t) * nodes);
    
    // start from initial node
    frontier.push({initial, initial, 0, estimated(initial)});
    froniter_set.emplace(initial);
    
    // a flag variable
    bool found = false;
    
    // frontier will be empty if goal is not in our search space
    while (!frontier.empty()) {
        // this is least weighted node
        auto current = frontier.top();
    
        // node number
        size_t current_node = std::get<0>(current);
        
        frontier.pop();
        froniter_set.erase(current_node);
        
        // record that this node has been visited
        visited.emplace(current_node);
        
        // we go to this node via last
        size_t from = std::get<1>(current);
        visit[current_node] = from;
        
        // goal test
        if ((found = (current_node == goal))) {
            break;
        } else {
            // iterate possible node
            for (size_t to = 0; to < nodes; to++) {
                // don't stay here
                // and if node to is reachable from current node
                if (to != current_node && graph[current_node][to] != std::numeric_limits<Weight>::max() && (visited.find(to) == visited.end())) {
                    // don't go back
                    if (froniter_set.find(to) == froniter_set.end()) {
                        // push this state to frontier
                        frontier.push({to, current_node, std::get<2>(current) + graph[current_node][to], estimated(to)});
                        froniter_set.emplace(to);
                    }
                }
            }
        }
    }
    
    std::vector<size_t> trace;
    if (found) {
        // build trace
        std::stack<size_t> trace_stack;
        size_t from = goal;
        trace_stack.push(from);
        while (from != initial) {
            from = visit[from];
            trace_stack.push(from);
        }
        
        // move trace into std::vector
        while (!trace_stack.empty()) {
            trace.emplace_back(trace_stack.top());
            trace_stack.pop();
        }
    }
    free((void *)visit);
    
    return trace;
}

#endif /* ASTAR_HPP */
