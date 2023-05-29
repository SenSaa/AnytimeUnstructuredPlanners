using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARRTStar;

namespace Rendering
{
    public class DrawObstacles
    {
        public void Draw(Material material)
        {
            processObstacles(material);
        }

        private void processObstacles(Material material)
        {
            RectObstacle[] obstacles = RRTConfig.OBSTACLES;
            for (int i = 0; i < obstacles.Length; i++)
            {
                double width = obstacles[i].width;
                double length = obstacles[i].length;
                Position2D position_2D = obstacles[i].pos2D;
                Position2D centerPosition_2D = new Position2D(position_2D.x + length/2, position_2D.y + width/2);
                Vector3 position = MathHelpers.Pos2DToVector3(centerPosition_2D);
                renderObstacles(position, width, length, material);
            }
        }

        private void renderObstacles(Vector3 position, double width, double length, Material material)
        {
            GameObject obstacleObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obstacleObject.transform.position = position;
            obstacleObject.transform.localScale = new Vector3((float)length, 1, (float)width);
            obstacleObject.GetComponent<MeshRenderer>().material = material;
        }
    }
}