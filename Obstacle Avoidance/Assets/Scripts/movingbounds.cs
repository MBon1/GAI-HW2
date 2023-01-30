using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class movingbounds : MonoBehaviour
{
    private void OnTriggerEnter2D(Collider2D collision)
    {
        if (collision.GetComponent<Movement_3>() != null && collision.GetComponent<Drive>() == null)
        {
            collision.GetComponent<Movement_3>().obstacleAvoidance = Movement_3.ObstacleAvoidanceOperation.CollisionPrediction;
        }
    }

    private void OnTriggerExit2D(Collider2D collision)
    {
        if (collision.GetComponent<Movement_3>() != null && collision.GetComponent<Drive>() == null)
        {
            collision.GetComponent<Movement_3>().obstacleAvoidance = collision.GetComponent<Movement_3>().defaultOA;
        }
    }
}
