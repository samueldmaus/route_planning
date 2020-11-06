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
    return node->distance(*RoutePlanner::end_node);

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
bool CompareFValues(const RouteModel::Node *node_one, const RouteModel::Node *node_two)
{
    float f_value_one = node_one->g_value + node_one->h_value;
    float f_value_two = node_two->g_value + node_two->h_value;
    return f_value_one > f_value_two;
}

RouteModel::Node *RoutePlanner::NextNode() {
    // sort the open list using the compare function above
    sort(open_list.begin(), open_list.end(), CompareFValues);

    // node with the lowest f-value should be at the back of the open_list
    RouteModel::Node *lowest_sum_node = open_list.back();

    // get the lowest_sum_node from the back of the vector & return it
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
    // while the parent of the current_node isn't nullptr(basically for every node except the start node)
    // we're tracing our steps back from the end_node
    while(current_node->parent != nullptr)
    {
        // push the current_node into path_found vector
        path_found.push_back(*current_node);
        // set the parent as the parent of the current_node
        auto parent = *(current_node->parent);
        // add the distance from the current_node to it's parent
        distance += current_node->distance(parent);
        // set the current_node as the parent of the old current_node; start the process over again
        current_node = current_node->parent;
    }

    // this is for the start_node as it won't a parent
    path_found.push_back(*current_node);

    // reverse the path so it's in the right order
    std::reverse(path_found.begin(), path_found.end());

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
    // start from the start_node
    RouteModel::Node *current_node = start_node;

    // TODO: Implement your solution here.
    // get information for the current_node which is the start_node here &
    // push it into the open_list 
    current_node->g_value = 0.0f;
    current_node->h_value = CalculateHValue(current_node);
    current_node->parent = nullptr;
    current_node->visited = true;
    open_list.push_back(current_node);

    // while open_list isn't empty do: get the next node, check if it's the end_node,
    // it is then construct the path, else addneighbors for the curren_node
    while(!open_list.empty())
    {
        current_node = NextNode();
        if(current_node == end_node)
        {
            // construct the final path starting with the end node
            m_Model.path = ConstructFinalPath(end_node);
            break;
        }else
        {
            AddNeighbors(current_node);
        }
        
    }

}