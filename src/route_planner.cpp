#include "route_planner.h"
#include <algorithm>

using namespace std;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float TheHValue = node->distance(*RoutePlanner::end_node);
    return TheHValue;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // find the neighbors of the current node
    current_node->FindNeighbors();
    // populate current_node.neighbors vector with all the neighbors
    for(auto v : current_node->neighbors)
    {
        //set the parent
        v->parent = current_node;
        //calculate the h-value
        v->h_value = RoutePlanner::CalculateHValue(v);
        //calculate the g-value
        v->g_value = current_node->g_value + current_node->distance(*v);
        //set the visited boolean to true
        v->visited = true;
        //add the neighbor to the open_list
        open_list.emplace_back(v);
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

// Compare function to compare the f values of the two nodes
bool Compare(const RouteModel::Node *node_one, const RouteModel::Node *node_two)
{
    float f_value_one = node_one->g_value + node_one->h_value;
    float f_value_two = node_two->g_value + node_two->h_value;
    return f_value_one > f_value_two;
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the open list using the compare function above
    sort(open_list.begin(), open_list.end(), Compare);

    RouteModel::Node *lowest_sum_node = open_list.back();
    open_list.pop_back();
    return lowest_sum_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // make sure the parent node isn't empty
    while(current_node->parent != nullptr)
    {
        path_found.push_back(*current_node);
        auto parent = *(current_node->parent);
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    AddNeighbors(current_node);
    while(!open_list.empty())
    {
        current_node = NextNode();
        if(current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(end_node);
            break;
        }
    }

}