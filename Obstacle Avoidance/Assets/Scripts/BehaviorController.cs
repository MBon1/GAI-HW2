using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class BehaviorController : MonoBehaviour
{
    [SerializeField] Movement_3 movement;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            SceneManager.LoadScene("Init");
        }
        /*else if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            movement.movement = Movement_3.MovementOperation.Seek;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            movement.movement = Movement_3.MovementOperation.Flee;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            movement.movement = Movement_3.MovementOperation.Pursue;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            movement.movement = Movement_3.MovementOperation.Evade;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            movement.movement = Movement_3.MovementOperation.Wander;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha6))
        {
            movement.movement = Movement_3.MovementOperation.FollowPath;
        }
        else if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            movement.movement = Movement_3.MovementOperation.None;
        }*/
        else
        {
            return;
        }
    }
}
