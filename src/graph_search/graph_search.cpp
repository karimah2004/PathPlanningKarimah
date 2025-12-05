#include <iostream>
#include <cmath>
#include <queue>

#include <path_planning/utils/math_helpers.h>
#include <path_planning/utils/graph_utils.h>

#include <path_planning/graph_search/graph_search.h>

/**
 * General graph search instructions:
 *
 * First, define the correct data type to keep track of your visited cells
 * and add the start cell to it. If you need to initialize any properties
 * of the start cell, do that too.
 *
 * Next, implement the graph search function. Save the result in the path
 * variable defined for you.
 *
 * To visualize which cells are visited in the navigation webapp, save each
 * visited cell in the vector in the graph struct as follows:
 *      graph.visited_cells.push_back(c);
 * where c is a Cell struct corresponding to the visited cell you want to
 * visualize.
 *
 * The tracePath() function will return a path (which you should assign to
 * the path variable above) given the goal index, if you have kept track
 * of the parent of each node correctly and have implemented the
 * getParent() function. If you do not find a path, return an empty path
 * vector.
 */

std::vector<Cell> depthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement DFS.
     */
    /*converts the goal cell to index */
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    /*create a stack for dfs*/
    std::vector<int> stack;
    /* Push the start node onto the stack → first thing DFS will explore. Mark it as visited so you don't add it again.*/
    stack.push_back(start_idx);
    graph.nodes[start_idx].visited = true;
    graph.nodes[start_idx].parent = -1;

    /*start dfs loop*/
    while (!stack.empty())
    {
        /*get the last node added to the stack and remove it from the stack */
        int current_idx = stack.back();
        stack.pop_back();

        /*if its the goal, call tracepath and build the final path*/
        if (current_idx == goal_idx)
        {
            path = tracePath(goal_idx, graph);
            break;
        }

        /*Retrieves all valid neighbors of the current cell*/
        std::vector<int> neighbors = findNeighbors(current_idx, graph);
        for (int neighbor_idx : neighbors)
        {
            if (!graph.nodes[neighbor_idx].visited && !isIdxOccupied(neighbor_idx, graph))
            {
                stack.push_back(neighbor_idx);
                graph.nodes[neighbor_idx].visited = true;
                graph.nodes[neighbor_idx].parent = current_idx;
            }
        }
    }

    return path;
}

std::vector<Cell> breadthFirstSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement BFS.
     */
    int goal_idx = cellToIdx(goal.i, goal.j, graph);
    /*bfs uses a queue */
    std::queue<int> queue;
    /* pushes the starting node into the queue*/
    queue.push(start_idx);
    /* marking starting node as visited and setting it as the parent node to avoid infinite loop */
    graph.nodes[start_idx].visited = true;
    graph.nodes[start_idx].parent = -1;
    /* starting bfs loop*/
    while (!queue.empty())
    {
        /*take the node thats been waiting longest and remove it from queue */
        int current_idx = queue.front();
        queue.pop();

        /* record it for visualization purposes*/
        graph.visited_cells.push_back(idxToCell(current_idx, graph));

        /* if the node being explored is the goal, stop the loop*/
        if (current_idx == goal_idx)
        {
            path = tracePath(goal_idx, graph);
            break;
        }

        /*return all the neighbors of the current node */
        std::vector<int> neighbors = findNeighbors(current_idx, graph);
        for (int neighbor_idx : neighbors)
        {
            if (!graph.nodes[neighbor_idx].visited && !isIdxOccupied(neighbor_idx, graph))
            {
                queue.push(neighbor_idx);
                graph.nodes[neighbor_idx].visited = true;
                graph.nodes[neighbor_idx].parent = current_idx;
            }
        }
    }

    return path;
}

std::vector<Cell> aStarSearch(GridGraph &graph, const Cell &start, const Cell &goal)
{
    std::vector<Cell> path; // The final path should be placed here.

    initGraph(graph); // Make sure all the node values are reset.

    int start_idx = cellToIdx(start.i, start.j, graph);

    /**
     * TODO (P3): Implement A-star search.
     */

    return path;
}
