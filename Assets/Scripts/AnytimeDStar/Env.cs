using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace ADStar
{
    public class Env
    {

        public int x_range;
        public int y_range;
        public List<Tuple<float, float>> motions;
        public HashSet<Tuple<float, float>> obs;

        public Env()
        {
            // - Env map size
            this.x_range = 200;
            this.y_range = 200;

            // - Types of movement grid cell map env
            this.motions = new List<Tuple<float, float>> {
                new Tuple<float, float>(-1, -1),
                new Tuple<float, float>(-1, 0),
                new Tuple<float, float>(-1, 1),
                new Tuple<float, float>(0, -1),
                new Tuple<float, float>(0, 1),
                new Tuple<float, float>(1, -1),
                new Tuple<float, float>(1, 0),
                new Tuple<float, float>(1, 1),
            };

            this.obs = new HashSet<Tuple<float, float>>();
        }

        public void update_obs(HashSet<Tuple<float, float>> obs)
        {
            this.obs = obs;
        }

    }
}