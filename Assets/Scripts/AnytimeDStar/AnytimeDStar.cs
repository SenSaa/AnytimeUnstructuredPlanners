// C#/Unity port of:
// https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_2D/Anytime_D_star.py

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using MoreLinq;

namespace ADStar
{
    public class AnytimeDStar
    {

        private Tuple<float, float> s_start;
        private Tuple<float, float> s_goal;
        private string heuristic_type;
        private Env env;
        private List<Tuple<float, float>> u_set;
        private HashSet<Tuple<float, float>> obs;
        private float x;
        private float y;
        private Dictionary<Tuple<float, float>, double> g;
        private Dictionary<Tuple<float, float>, double> rhs;
        private Dictionary<Tuple<float, float>, List<double>> OPEN;
        private double eps;
        private HashSet<Tuple<float, float>> CLOSED;
        private Dictionary<Tuple<float, float>, int> INCONS;
        private HashSet<Tuple<float, float>> visited;
        private int count;
        private int count_env_change;
        private HashSet<Tuple<float, float>> obs_add;
        private HashSet<Tuple<float, float>> obs_remove;
        private string title;

        private int max_iteration = 500;
        private List<Tuple<float, float>> finalPath;
        private HashSet<Tuple<float, float>> visitedStates;
        private List<Tuple<float, float>> topKeys;
        private int iteration = 0;
        public bool onPressFlag = false;
        private string s_Str = "";

        public AnytimeDStar(Tuple<float, float> s_start, Tuple<float, float> s_goal, double eps, string heuristic_type, HashSet<Tuple<float, float>> obs)
        {
            this.s_start = s_start;
            this.s_goal = s_goal;
            this.heuristic_type = heuristic_type;
            this.env = new Env();
            this.u_set = this.env.motions;
            this.obs = obs;
            this.x = this.env.x_range;
            this.y = this.env.y_range;
            this.g = new Dictionary<Tuple<float, float>, double>()
            {
            };
            this.rhs = new Dictionary<Tuple<float, float>, double>()
            {
            };
            this.OPEN = new Dictionary<Tuple<float, float>, List<double>>()
            {
            };

            // - Initialise cost of all nodes in the cell grid env map as infinity!
            foreach (var i in Enumerable.Range(-this.env.x_range/2, this.env.x_range))
            {
                foreach (var j in Enumerable.Range(-this.env.y_range/2, this.env.y_range))
                {
                    this.rhs[new Tuple<float, float>(i, j)] = double.PositiveInfinity;
                    this.g[new Tuple<float, float>(i, j)] = double.PositiveInfinity;
                }
            }

            this.rhs[this.s_goal] = 0.0;
            this.eps = eps;
            this.OPEN[this.s_goal] = this.Key(this.s_goal);
            this.CLOSED = new HashSet<Tuple<float, float>>();
            this.INCONS = new Dictionary<Tuple<float, float>, int>();
            this.visited = new HashSet<Tuple<float, float>>();
            this.count = 0;
            this.count_env_change = 0;
            this.obs_add = new HashSet<Tuple<float, float>>();
            this.obs_remove = new HashSet<Tuple<float, float>>();
            this.title = "Anytime D*: Small changes";
        }

        public void run()
        {
            this.ComputeOrImprovePath();

            visitedStates = this.visited;
            finalPath = this.extract_path();
            Debug.Log("path: " + GeneralHelpers.ReturnListAsSingleString(finalPath));
            this.visited = new HashSet<Tuple<float, float>>();
            int _iterations = 0;
            
            while (true)
            {
                if (this.eps <= 1.0)
                {
                    break;
                }
                this.eps -= 0.5;
                Debug.Log("OPEN: " + GeneralHelpers.ReturnDictAsSingleString(OPEN));
                Debug.Log("INCONS: " + GeneralHelpers.ReturnDictAsSingleString(INCONS));
                Debug.Log("CLOSED: " + GeneralHelpers.ReturnSetAsSingleString(CLOSED));
                foreach (var INCONS_element in INCONS)
                {
                    if (!OPEN.ContainsKey(INCONS_element.Key))
                    {
                        OPEN.Add(INCONS_element.Key, new List<double>(INCONS_element.Value));
                    }
                }
                foreach (var s in this.OPEN.Keys.ToList())
                {
                    OPEN.Remove(s);
                    OPEN.Add(s,Key(s));
                }
                this.CLOSED = new HashSet<Tuple<float, float>>();
                _iterations += 1;
                Debug.Log("_" + _iterations);
                this.ComputeOrImprovePath();
                visitedStates = this.visited;
                finalPath = this.extract_path();
                Debug.Log("path: " + GeneralHelpers.ReturnListAsSingleString(finalPath));
                this.visited = new HashSet<Tuple<float, float>>();
            }
        }

        public void ComputeOrImprovePath()
        {
            Debug.Log("i = " + iteration);
            while (OPEN.Count > 0 && iteration < max_iteration)
            {
                try
                {

                    var _tup_1 = this.TopKey();
                    var s = _tup_1.Item1;
                    var v = _tup_1.Item2;
                    s_Str += s + "-";
                    if (GeneralHelpers.CheckIfList1ExceedsList2(v, Key(this.s_start)) && this.rhs[this.s_start] == this.g[this.s_start])
                    {
                        break;
                    }
                    this.OPEN.Remove(s);
                    this.visited.Add(s);
                    if (this.g[s] > this.rhs[s])
                    {
                        this.g[s] = this.rhs[s];
                        this.CLOSED.Add(s);
                        foreach (var sn in this.get_neighbor(s))
                        {
                            this.UpdateState(sn);
                        }
                    }
                    else
                    {
                        this.g[s] = double.PositiveInfinity;
                        foreach (var sn in this.get_neighbor(s))
                        {
                            this.UpdateState(sn);
                        }
                        this.UpdateState(s);
                    }
                    
                    iteration++;
                }
                catch(Exception e)
                {
                    Debug.LogError(e);
                }
            }

        }

        public void UpdateState(Tuple<float, float> s)
        {
            if (!s.Equals(s_goal))
            {
                this.rhs[s] = double.PositiveInfinity;
                foreach (var x in this.get_neighbor(s))
                {
                    this.rhs[s] = Math.Min(this.rhs[s], this.g[x] + this.cost(s, x));
                }
            }
            if (this.OPEN.ContainsKey(s))
            {
                this.OPEN.Remove(s);
                if (onPressFlag)
                {
                    Debug.Log("Remove this -> (s) <- from OPEN -> :" + s);
                }
            }
            if (this.g[s] != this.rhs[s])
            {
                if (!this.CLOSED.Contains(s))
                {
                    OPEN.Remove(s);
                    OPEN.Add(s, Key(s));
                }
                else
                {
                    INCONS.Remove(s);
                    INCONS.Add(s, 0);
                }
            }
        }

        public List<double> Key(Tuple<float, float> s)
        {
            if (this.g[s] > this.rhs[s])
            {
                return new List<double> {
                        this.rhs[s] + this.eps * this.h(this.s_start, s),
                        this.rhs[s]
                    };
            }
            else
            {
                return new List<double> {
                        this.g[s] + this.h(this.s_start, s),
                        this.g[s]
                    };
            }
        }

        //         :return: return the min key and its value.
        public Tuple<Tuple<float, float>, List<double>> TopKey()
        {
            var s = OPEN.Aggregate((l, r) =>
                l.Value[0] < r.Value[0] || (l.Value[0] == r.Value[0] && l.Value[1] < r.Value[1]) ? l : r)
                .Key;

            return Tuple.Create(s, this.OPEN[s]);
        }

        public double h(Tuple<float, float> s_start, Tuple<float, float> s_goal)
        {
            var heuristic_type = this.heuristic_type;
            if (heuristic_type.Equals("manhattan"))
            {
                return Math.Abs(s_goal.Item1 - s_start.Item1) + Math.Abs(s_goal.Item2 - s_start.Item2);
            }
            else
            {
                return MathHelpers.Hypotenuse(s_goal.Item1 - s_start.Item1, s_goal.Item2 - s_start.Item2);
            }
        }

        //         Calculate Cost for this motion
        //         :param s_start: starting node
        //         :param s_goal: end node
        //         :return:  Cost for this motion
        //         :note: Cost function could be more complicate!
        public double cost(Tuple<float, float> s_start, Tuple<float, float> s_goal)
        {
            if (this.is_collision(s_start, s_goal))
            {
                return double.PositiveInfinity;
            }
            return MathHelpers.Hypotenuse(s_goal.Item1 - s_start.Item1, s_goal.Item2 - s_start.Item2);
        }

        public bool is_collision(Tuple<float, float> s_start, Tuple<float, float> s_end)
        {
            object s2;
            object s1;
            if (this.obs.Contains(s_start) || this.obs.Contains(s_end))
            {
                return true;
            }
            if (s_start.Item1 != s_end.Item1 && s_start.Item2 != s_end.Item2)
            {
                if (s_end.Item1 - s_start.Item1 == s_start.Item2 - s_end.Item2)
                {
                    s1 = (Math.Min(s_start.Item1, s_end.Item1), Math.Min(s_start.Item2, s_end.Item2));
                    s2 = (Math.Max(s_start.Item1, s_end.Item1), Math.Max(s_start.Item2, s_end.Item2));
                }
                else
                {
                    s1 = (Math.Min(s_start.Item1, s_end.Item1), Math.Max(s_start.Item2, s_end.Item2));
                    s2 = (Math.Max(s_start.Item1, s_end.Item1), Math.Min(s_start.Item2, s_end.Item2));
                }
                if (this.obs.Contains(s1) || this.obs.Contains(s2))
                {
                    return true;
                }
            }
            return false;
        }

        public HashSet<Tuple<float, float>> get_neighbor(Tuple<float, float> s)
        {
            var nei_list = new HashSet<Tuple<float, float>>();
            foreach (var u in this.u_set)
            {
                var s_next = MathHelpers.SumIntTuples(s, u);
                if (!this.obs.Contains(s_next))
                {
                    nei_list.Add(s_next);
                }
            }
            return nei_list;
        }

        //         Extract the path based on the PARENT set.
        //         :return: The planning path
        public List<Tuple<float, float>> extract_path()
        {
            int _i = 0;
            var path = new List<Tuple<float, float>> {
                    this.s_start
                };
            var s = this.s_start;
            foreach (var k in Enumerable.Range(0, 100))
            {
                try
                {
                    var g_list = new Dictionary<Tuple<float, float>, double>
                    {
                    };
                    foreach (var x in this.get_neighbor(s))
                    {
                        if (!this.is_collision(s, x))
                        {
                            g_list[x] = this.g[x];
                        }
                    }

                    string g_list_Str = "";
                    if (_i == 0)
                    {
                        foreach (var g in g_list.Keys)
                        {
                            g_list_Str += "K: " + g.Item1 + ", " + g.Item1 + " -> V: " + g_list[g] + " | ";
                        }
                        Debug.Log("g_list_Str: " + g_list_Str);
                    }

                    s = g_list.Aggregate((l, r) => l.Value < r.Value ? l : r).Key;
                    if (path.Contains(s)) { continue; } // * Prevent revisits!
                    path.Add(s);
                    if (s.Equals(s_goal))
                    {
                        break;
                    }
                    _i++;
                }
                catch(Exception e) { Debug.LogError(e); }
            }
            return path.ToList();
        }


        public List<Tuple<float, float>> getPath()
        {
            return finalPath;
        }
        public HashSet<Tuple<float, float>> getVisitedStates()
        {
            return visitedStates;
        }
        public List<Tuple<float, float>> getTopKeys()
        {
            return topKeys;
        }
        public HashSet<Tuple<float, float>> getObs()
        {
            return obs;
        }


        // ---

        public void on_press(float x, float y)
        {
            onPressFlag = true;
            if (x < 0 || x > this.x - 1 || y < 0 || y > this.y - 1)
            {
                Debug.LogError("Please choose right area!");
            }
            else
            {
                this.count_env_change += 1;
                x = Convert.ToInt32(x);
                y = Convert.ToInt32(y);
                // *
                Tuple<float, float> newGoal = Tuple.Create(x, y);
                Debug.Log("Change position: s = " + x + ", " + "y = " + y);
                // for small changes
                if (this.title == "Anytime D*: Small changes")
                {
                    if (!this.obs.Contains(newGoal))
                    {
                        Debug.Log("!this.obs.Contains(newGoal)");
                        this.obs.Add(newGoal);
                        this.g[newGoal] = double.PositiveInfinity;
                        this.rhs[newGoal] = double.PositiveInfinity;
                    }
                    else
                    {
                        Debug.Log("this.obs.Contains(newGoal)");
                        this.obs.Remove(newGoal);
                        this.UpdateState(newGoal);
                    }

                    foreach (var sn in this.get_neighbor(newGoal))
                    {
                        //Debug.Log("sn -> " + sn);
                        this.UpdateState(sn);
                    }
                    while (true)
                    {
                        if (this.INCONS.Count == 0)
                        {
                            break;
                        }
                        foreach (var INCONS_element in INCONS)
                        {
                            if (!OPEN.ContainsKey(INCONS_element.Key))
                            {
                                OPEN.Add(INCONS_element.Key, new List<double>(INCONS_element.Value));
                            }
                            else
                            {
                                OPEN.Remove(INCONS_element.Key);
                                OPEN.Add(INCONS_element.Key, new List<double>(INCONS_element.Value));
                            }
                        }
                        foreach (var s in this.OPEN.Keys.ToList())
                        {
                            OPEN.Remove(s);
                            OPEN.Add(s, Key(s));
                        }
                        this.CLOSED = new HashSet<Tuple<float, float>>();
                        this.ComputeOrImprovePath();
                        visitedStates = this.visited;
                        finalPath = this.extract_path();
                        this.visited = new HashSet<Tuple<float, float>>();
                        if (this.eps <= 1.0)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    if (!this.obs.Contains(newGoal))
                    {
                        this.obs.Add(newGoal);
                        this.obs_add.Add(newGoal);
                        if (this.obs_remove.Contains(newGoal))
                        {
                            this.obs_remove.Remove(newGoal);
                        }
                    }
                    else
                    {
                        this.obs.Remove(newGoal);
                        this.obs_remove.Add(newGoal);
                        if (this.obs_add.Contains(newGoal))
                        {
                            this.obs_add.Remove(newGoal);
                        }
                    }
                    if (this.count_env_change >= 15)
                    {
                        this.count_env_change = 0;
                        this.eps += 2.0;
                        foreach (var s in this.obs_add)
                        {
                            this.g[Tuple.Create(x, y)] = double.PositiveInfinity;
                            this.rhs[Tuple.Create(x, y)] = double.PositiveInfinity;
                            foreach (var sn in this.get_neighbor(s))
                            {
                                this.UpdateState(sn);
                            }
                        }
                        foreach (var s in this.obs_remove)
                        {
                            foreach (var sn in this.get_neighbor(s))
                            {
                                this.UpdateState(sn);
                            }
                            this.UpdateState(s);
                        }
                        while (true)
                        {
                            if (this.eps <= 1.0)
                            {
                                break;
                            }
                            this.eps -= 0.5;
                            foreach (var INCONS_element in INCONS)
                            {
                                if (!OPEN.ContainsKey(INCONS_element.Key))
                                {
                                    OPEN.Add(INCONS_element.Key, new List<double>(INCONS_element.Value));
                                }
                                else
                                {
                                    OPEN.Remove(INCONS_element.Key);
                                    OPEN.Add(INCONS_element.Key, new List<double>(INCONS_element.Value));
                                }
                            }
                            foreach (var s in this.OPEN.Keys.ToList())
                            {
                                OPEN.Remove(s);
                                OPEN.Add(s, Key(s));
                            }
                            this.CLOSED = new HashSet<Tuple<float, float>>();
                            this.ComputeOrImprovePath();
                            visitedStates = this.visited;
                            finalPath = this.extract_path();
                            this.visited = new HashSet<Tuple<float, float>>();
                        }
                    }
                }
                onPressFlag = false;
            }
        }

    }

}
