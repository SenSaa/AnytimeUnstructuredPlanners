// https://github.com/danny45s/motion-planning

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ARRTStar
{

    public class RRTConfig
    {

        // Distance to move towards random points. This is effectively the max edge length.
        public static double DELTA = 4;

        // Constant multiplier for radius for nearby neighbors search for RRT*
        public static double GAMMA_RRT_STAR   // property
        {
            get { return DELTA * 100; }   // get method
        }

        // List of rectangular obstacles, ((bottom left corner), length, height))
        public static RectObstacle[] OBSTACLES = new RectObstacle[] { new RectObstacle(new Position2D(0, 20), 80, 20), 
                                                                        new RectObstacle(new Position2D(20, 60), 80, 20) ,
                                                                        new RectObstacle(new Position2D(-20, -60), 80, 20) ,
                                                                        new RectObstacle(new Position2D(-60, 20), 20, 80) ,
                                                                        new RectObstacle(new Position2D(-60, -80), 20, 80) ,
                                                                    };

        // -------------------------------------------------------------------------------------------------------------------------------------------
        // Bounds for search space
        public static int RIGHT_BOUND = 200;
        public static int BOTTOM_BOUND = 200;
        // -------------------------------------------------------------------------------------------------------------------------------------------

        // How fast the agent moves along a path
        public static double AGENT_SPEED = 0.01;

        // For rounding issues
        public static double EPSILON = 0.01;

        // How long to pause when animating
        public static double VIS_PAUSE_LENGTH = 0.01;

        // For anytime RRT*, how long each saved trajectory should be.
        // i.e. how many edges away to set the new root
        public static double ANYTIME_TRAJECTORY_LENGTH = 5;

        // Starting position of the agent
        public static Position2D AGENT_START_POSITION = new Position2D(10, 10);

        // Starting position of the goal ((bottom left), (top right))
        public static Position2D[] GOAL_POSITION = new Position2D[] { new Position2D(8, 8), new Position2D(12, 12) };

    }
}