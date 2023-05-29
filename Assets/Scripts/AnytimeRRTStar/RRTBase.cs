// https://github.com/danny45s/motion-planning

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
//using Utils;
using Common;

namespace ARRTStar
{

    public class RRTBase
    {

        public List<Tuple<double, double>> points;
        public Dictionary<Tuple<double, double>, Tuple<double, double>> parents;
        public Dictionary<Tuple<double, double>, List<Tuple<double, double>>> edges;
        public Position2D[] goal;
        public Position2D agent_pos;

        // For visualization
        public Dictionary<Tuple<double, double>, List<Tuple<double, double>>> edges_to_lines;
        public List<Tuple<double, double>> lines_to_goal;
        public List<Tuple<double, double>> pathPoints;
        public Vector3 agent_viz;
        public Vector3 goal_viz;


        public RRTBase(Position2D start, Position2D[] goal)
        {
            this.points = new List<Tuple<double, double>> { (Tuple.Create(start.x, start.y)) };
            this.parents = new Dictionary<Tuple<double, double>, Tuple<double, double>>();
            edges = new Dictionary<Tuple<double, double>, List<Tuple<double, double>>>();
            this.goal = goal;
            this.agent_pos = start;

            // For visualization
            this.edges_to_lines = new Dictionary<Tuple<double, double>, List<Tuple<double, double>>>();
            this.lines_to_goal = new List<Tuple<double, double>>();
            this.pathPoints = new List<Tuple<double, double>>();
            this.agent_viz = Vector3.zero;
            this.goal_viz = Vector3.zero;
        }

        // Perform one full step of adding a point to the RRT.
        // Includes choosing a random point, steering towards it,
        // rewiring if applicable, etc.
        // Check RRT* =>
        public void step() { }

        // Return whether the given point is within the boundaries of the goal.
        // point: (x, y) tuple representing the point
        // return: whether the point is in the goal
        public bool is_in_goal(Tuple<double, double> point)
        {
            return goal[0].x <= point.Item1 && point.Item1 <= goal[1].x && goal[0].y <= point.Item2 && point.Item2 <= goal[1].y;
        }

        // Return the cost from the root of a point in the tree.
        // Cost is the sum of edge lengths from the root to this point.
        // point: (x, y) tuple representing the point
        // return: cost from the root to this point along the tree
        public double get_cost(Tuple<double, double> point)
        {
            double cost = 0;
            while (this.parents.ContainsKey(point))
            {
                var parent = parents[point];
                cost += calculate_distance(point, parent);
                point = parent;
            }
            return cost;
        }

        // Return whether this point is free of obstacles
        // point: (x, y) tuple representing the point
        // return: False if the point collides with an obstacle, otherwise True
        // TODO: more collision checks along an edge as opposed to just at one point
        public bool obstacle_free(Tuple<double, double> point)
        {
            foreach (var obstacle in RRTConfig.OBSTACLES)
            {
                var _tup_1 = obstacle.pos2D;
                var left = _tup_1.x;
                var top = _tup_1.y;
                var right = left + obstacle.length;
                var bottom = top + obstacle.width;
                if (left <= point.Item1 && point.Item1 <= right && top <= point.Item2 && point.Item2 <= bottom)
                {
                    return false;
                }
            }
            return true;
        }

        // Return the lowest-cost path to the goal from the root of the RRT.
        // return: the shortest path, comprised of a list of points
        // in order from root to goal.
        public List<Tuple<double, double>> best_path_to_goal()
        {
            var points_in_goal = (from point in points
                                  where this.is_in_goal(point)
                                  select point).ToList();
            if (points_in_goal.Count < 1 || points_in_goal == null)
            {
                return null;
            }
            var cur_point = MathHelpers.MinBy(points_in_goal, (x) => get_cost(x));
            Debug.Log("Path cost:" + this.get_cost(cur_point));
            List<Tuple<double, double>> path = new List<Tuple<double, double>> {
                    cur_point
                };
            while (this.parents.ContainsKey(cur_point))
            {
                cur_point = this.parents[cur_point];
                path.Add(cur_point);
            }
            path.Reverse();
            return path;
        }

        // Note: path is expected to be in reverse order for computational efficiency.
        // Move the agent "to_travel" units along the path, updating the
        // path to remove points as they're reached.
        // path: list of point tuples in reverse order
        // to_travel: distance left to travel, used for recursive calls if
        // the next point is less than to_travel away from the agent
        public void take_step_along_path(List<Tuple<double, double>> path, double to_travel = 0.1)
        {
            to_travel = RRTConfig.AGENT_SPEED;
            Tuple<double, double> agentPosDoubleTuple = MathHelpers.Pos2DToDoubleTuple(agent_pos);

            if (path.Count < 1 || path == null)
            {
                return;
            }
            var point = path[path.Count - 1];
            var distance = this.calculate_distance(agentPosDoubleTuple, point);
            if (distance <= to_travel)
            {
                updateAgentPos(agentPosDoubleTuple, point);
                path.RemoveAt(path.Count - 1); // pop last item
                this.take_step_along_path(path, to_travel - distance);
            }
            else
            {
                var dist_x = point.Item1 - this.agent_pos.x;
                var dist_y = point.Item2 - this.agent_pos.y;
                Tuple<double, double> new_point = Tuple.Create(this.agent_pos.x + to_travel / distance * dist_x, agent_pos.y + to_travel / distance * dist_y);
                updateAgentPos(agentPosDoubleTuple, new_point);
            }
        }


        // Keep moving the agent along the path until it's reached the end.
        // path: list of point tuples in reverse order
        // keep_stepping: Whether to call the RRT's step function while moving.
        // Useful for anytime planning.
        public void move_along_path_until_done(List<Tuple<double, double>> path, bool keep_stepping = false, bool visualize = true)
        {
            while (path.Count > 1 || path != null)
            {
                if (path.Count < 1) { break; }

                take_step_along_path(path);

                if (keep_stepping)
                {
                    step();
                }
            }
        }


        // Calculate the Euclidean distance between two points
        // a: first (x, y) point
        // b: second (x, y) point
        // return: Euclidean distance between a and b
        public double calculate_distance(Tuple<double, double> a, Tuple<double, double> b)
        {
            return Math.Sqrt(Math.Pow((b.Item2 - a.Item2), 2) + Math.Pow((b.Item1 - a.Item1), 2));
        }

        // Return the closest point in the RRT to the given new point
        // new_point: (x, y) tuple representing a new point
        // return: (x, y) tuple of a point in the tree nearest to this point.
        public Tuple<double, double> nearest_neighbor(Tuple<double, double> new_point)
        {
            return this.points.MinBy((x) => calculate_distance(new_point, x));
        }

        // Returns a new point distance DELTA from the source, towards the destination
        // source: first (x, y) point
        // destination: second (x, y) point to steer from source to
        // return: a new (x, y) point distance DELTA closer to destination, from source
        public Tuple<double, double> get_new_point(Tuple<double, double> source, Tuple<double, double> destination)
        {
            var distance = this.calculate_distance(source, destination);
            if (distance <= RRTConfig.DELTA)
            {
                if (!obstacle_free(destination))
                {
                    return null;
                }
                return destination;
            }
            double dist_x = destination.Item1 - source.Item1;
            double dist_y = destination.Item2 - source.Item2;
            Tuple<double, double> new_point = Tuple.Create(source.Item1 + RRTConfig.DELTA / distance * dist_x, source.Item2 + RRTConfig.DELTA / distance * dist_y);
            if (!obstacle_free(new_point))
            {
                return null;
            }
            return new_point;
        }

        // Remove the current path to goal from the visualization,
        // and redraw a new one based on the current best path to the goal.
        public List<Tuple<double, double>> redraw_path_to_goal()
        {
            try
            {
                lines_to_goal.RemoveRange(0, lines_to_goal.Count);
                lines_to_goal = new List<Tuple<double, double>>();
                pathPoints.RemoveRange(0, pathPoints.Count);
                pathPoints = new List<Tuple<double, double>>();
                List<Tuple<double, double>> path = best_path_to_goal();
                for (int i = 0; i < path.Count - 1; i++)
                {
                    Tuple<double, double> start = path[i];
                    Tuple<double, double> end = path[i + 1];
                    Tuple<double, double>[] line = {
                        new Tuple<double,double>(start.Item1, end.Item1),
                        new Tuple<double,double>(start.Item2, end.Item2)
                    };
                    Vector3 from = MathHelpers.DoubleTupleToVector3(line[0]);
                    Vector3 to = MathHelpers.DoubleTupleToVector3(line[1]);
                    Debug.DrawLine(from, to, Color.black, 10);
                    lines_to_goal.AddRange(line);

                    pathPoints.Add(path[i]);
                }
            }
            catch (Exception e) { Debug.LogError(e); }
            return lines_to_goal;
        }


        private void updateAgentPos(Tuple<double, double> agentPosDoubleTuple, Tuple<double, double> target)
        {
            agentPosDoubleTuple = target;
            updatAgentPos(target);
            invokeRRTPathVisualsUpdate();
        }
        private void updatAgentPos(Tuple<double, double> target)
        {
            agent_pos = MathHelpers.DoubleTupleToPos2D(target);
        }


        private void invokeRRTPathVisualsUpdate()
        {
            EventBroker.AgentPosUpdateEventArgs e = new EventBroker.AgentPosUpdateEventArgs
            {
                AgentPos = new Vector3((float)agent_pos.x, 0, (float)agent_pos.y)
            };
            EventBroker.InvokeAgentPosUpdate(this, e);
        }

    }

}