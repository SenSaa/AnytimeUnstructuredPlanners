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
    public class RRTStar : RRTBase
    {

        private System.Random rand = new System.Random();
        Tuple<double, double> random_point;
        Tuple<double, double> new_point;
        Tuple<double, double> best_neighbor;
        private Vector3 randomPoint;
        private Vector3 newPoint;

        public RRTStar(Position2D start, Position2D[] goal) : base(start, goal)
        {

        }

        //         Given a new point and a list of neighbors, return the one that minimizes
        //         cost(neighbor) + distance(neighbor, new_point)
        //         new_point: (x, y) tuple describing the new point
        //         neighbors: list of points from which we want to choose
        //         return: The point in neighbors that minimizes distance through it to new_point
        public Tuple<double, double> min_cost_neighbor(Tuple<double, double> new_point, List<Tuple<double, double>> neighbors)
        {
            return MathHelpers.MinBy(neighbors, (neighbor) => base.get_cost(neighbor) + base.calculate_distance(neighbor, new_point));
        }

        //         Finds all points in the tree that are within
        //         GAMMA_RRT_STAR * sqrt(log_2(number of points) / number of points)
        //         distance from the new point.
        //         new_point: (x, y) tuple describing the new point
        //         return: List of points in the RRT this distance from the new point
        public List<Tuple<double, double>> near_neighbors(Tuple<double, double> new_point)
        {
            if (this.points.Count == 1)
            {
                return this.points;
            }
            List<Tuple<double, double>> neighbors = new List<Tuple<double, double>>();
            var distance_cutoff = Math.Min(RRTConfig.GAMMA_RRT_STAR * Math.Pow(Math.Log(this.points.Count, 2) / this.points.Count, 0.5), RRTConfig.DELTA);
            foreach (var point in this.points)
            {
                var dist = this.calculate_distance(new_point, point);
                if (dist - RRTConfig.EPSILON <= distance_cutoff)
                {
                    neighbors.Add(point);
                }
            }
            return neighbors;
        }

        //         Return neighbors that should be rewired given a list of neighbors and a new
        //         point to rewire through. Neighbors should be rewired if:
        //         cost(new_point) + distance(new_point, neighbor) < cost(neighbor)
        //         neighbors: list of points from which we choose a subset to rewire
        //         new_point: (x, y) tuple describing the new point
        //         return: a list of points describing the points from neighbors that should
        //         be rewired.
        public List<Tuple<double, double>> neighbors_to_rewire(List<Tuple<double, double>> neighbors, Tuple<double, double> new_point)
        {
            List<Tuple<double, double>> resulting_neighbors = new List<Tuple<double, double>>();
            foreach (var neighbor in neighbors)
            {
                if (this.get_cost(new_point) + this.calculate_distance(new_point, neighbor) < this.get_cost(neighbor))
                {
                    resulting_neighbors.Add(neighbor);
                }
            }
            return resulting_neighbors;
        }

        //         Given a neighbor and a new point (both points in the RRT),
        //         rewire the neighbor through the new point. The new point becomes
        //         its new parent, and we remove the edge between the neigbhbor and
        //         its previous parent.
        //         neighbor: (x, y) tuple representing point to rewire through new point
        //         new_point: (x, y) tuple representing new parent of the neighbor
        public void rewire_neighbor_through_new_point(Tuple<double, double> neighbor, Tuple<double, double> new_point, bool visualize = true)
        {
            var parent = this.parents[neighbor];
            this.parents[neighbor] = new_point;

            if (edges.ContainsKey(parent))
            {
                this.edges[parent].Remove(neighbor);
            }
            
            if (!edges.ContainsKey(new_point)) { edges.Add(new_point, new List<Tuple<double, double>>()); }
            if (!edges[new_point].Contains(neighbor)) { edges[new_point].Add(neighbor); }
        }


        public bool step(bool visualize = true)
        {
            // - Search space region
            random_point = Tuple.Create((rand.NextDouble() * 2.0 - 1.0) * RRTConfig.RIGHT_BOUND / 2, (rand.NextDouble() * 2.0 - 1.0) * RRTConfig.BOTTOM_BOUND / 2);

            var nearest = this.nearest_neighbor(random_point);
            new_point = this.get_new_point(nearest, random_point);
            invokeNewPointUpdate();
            if (new_point == null)
            {
                return false;
            }

            // get nearby neighbors to the new point, including nearest
            near_neighbors(new_point).Add(nearest);
            List<Tuple<double, double>> nearby = near_neighbors(new_point);
            best_neighbor = this.min_cost_neighbor(new_point, nearby);
            invokeLineToBestNeighbour();
            if (!edges.ContainsKey(best_neighbor)) { edges.Add(best_neighbor, new List<Tuple<double, double>>()); }
            if (!edges[best_neighbor].Contains(new_point)) { edges[best_neighbor].Add(new_point); }
            this.parents[new_point] = best_neighbor;
            this.points.Add(new_point);

            // Rewire neighbors that would have a better cost if they went through our new point instead.
            foreach (var neighbor in neighbors_to_rewire(nearby, new_point))
            {
                if (neighbors_to_rewire(nearby, new_point).IndexOf(neighbor) == neighbors_to_rewire(nearby, new_point).Count - 1)
                { continue; }
                this.rewire_neighbor_through_new_point(neighbor, new_point, visualize: visualize);
            }
            return this.is_in_goal(new_point);
        }

        private void invokeRandomPointUpdate()
        {
            randomPoint = MathHelpers.DoubleTupleToVector3(random_point);
            EventBroker.RandomPointEventArgs e = new EventBroker.RandomPointEventArgs
            {
                RandomPoint = randomPoint
            };
            EventBroker.InvokeRandomPointUpdate(this, e);
        }

        private void invokeNewPointUpdate()
        {
            if (new_point == null) { return; }
            newPoint = MathHelpers.DoubleTupleToVector3(new_point);
            EventBroker.NewPointEventArgs e = new EventBroker.NewPointEventArgs
            {
                NewPoint = newPoint
            };
            EventBroker.InvokeNewPointUpdate(this, e);
        }

        private void invokeLineToBestNeighbour()
        {
            Vector3 bestNeighbor = MathHelpers.DoubleTupleToVector3(best_neighbor);
            EventBroker.LineToBestNeighbourEventArgs e = new EventBroker.LineToBestNeighbourEventArgs
            {
                NewPoint = newPoint,
                BestNeighbour = bestNeighbor
            };
            EventBroker.InvokeLineToBestNeighbour(this, e);
        }

    }
}