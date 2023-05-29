// C#/Unity port of:
//https://github.com/danny45s/motion-planning

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace ARRTStar
{

    public class AnytimeRRTStar : RRTStar
    {

        private List<List<Vector3>> lines_from_start;
        private List<List<Vector3>> lines_from_current;
        private Stack<Tuple<double, double>> stack = new Stack<Tuple<double, double>>();
        Tuple<double, double> new_root;

        public AnytimeRRTStar(Position2D start, Position2D[] goal) : base(start, goal)
        {
            lines_from_start = new List<List<Vector3>>();
            lines_from_current = new List<List<Vector3>>();
        }

        public void step_until_found_goal()
        {
            // Keep running steps of the RRT until the goal is found.
            bool found_goal = false;
            while (!found_goal)
            {
                found_goal = step();
            }
        }

        // *
        public void update_and_draw_current_trajectory(List<Tuple<double, double>> trajectory)
        {
            // Draw edges in the current trajectory.Also update RRT's visualization
            // of the trajectory from the start to include the most recent trajectory.
            // That trajectory can be deleted when a new goal is chosen, but the current one
            // remains drawn.
            // trajectory: List of points to draw edges from and to update the RRT's
            // internal tracker of drawn edges.
            lines_from_start.AddRange(lines_from_current);
            lines_from_current = new List<List<Vector3>>();
            for (int i=0; i < trajectory.Count - 1; i++)
            {
                Tuple<double, double> start = trajectory[i];
                Tuple<double, double> end = trajectory[i + 1];
                var line = new List<Vector3> { new Vector3((float)start.Item1, (float)end.Item1), new Vector3((float)start.Item2, (float)end.Item2) };
                Debug.DrawLine(new Vector3((float)start.Item1, 0, (float)end.Item1), new Vector3((float)start.Item2, 0, (float)end.Item2), Color.blue, 10);
                lines_from_current.Add(line);
            }
        }
        

        public void prune_tree_and_set_new_root(List<Tuple<double, double>> trajectory)
        {
            try
            {
                // Given a trajectory, set the new root to the end of the trajectory by
                // deleting all branches of the tree that originate on that trajectory.
                // trajectory: List of points from which to prune branches, so that the
                // new root becomes the end of the trajectory.
                new_root = trajectory[trajectory.Count - 1];
                stack.Push(trajectory[0]);

                while (stack.Count > 0)
                {
                    // Remove and return the last element.
                    Tuple<double, double> cur = stack.Peek();
                    stack.Pop();

                    if (parents.ContainsKey(cur))
                    {
                        Tuple<double, double> parent = parents[cur];

                        parents.Remove(cur);
                        edges[parent].Remove(cur);
                    }
                    if (cur != new_root)
                    {
                        points.Remove(cur);
                        if (edges.ContainsKey(cur))
                        {
                            foreach(var edge_cur in edges[cur])
                            {
                                stack.Push(edge_cur);
                            }
                        }
                    }
                }
            }
            catch (Exception e) { Debug.LogError(e); }
        }

    }

}