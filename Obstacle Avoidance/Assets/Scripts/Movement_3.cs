using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(Rigidbody2D))]
public class Movement_3 : MonoBehaviour
{
    // How the agent should move
    public MovementOperation movement = MovementOperation.None;
    MovementOperation prevMovement;

    // Target
    [SerializeField] GameObject targetGO = null;

    // Holds the kinematic data for the character and target
    Rigidbody2D characterRb;
    Rigidbody2D targetRb;
    Kinematic character;    // Must convert characterRb to Kinematic and assign it to character each frame
    Kinematic target;       // Must convert targetRb to Kinematic and assign it to target each frame

    [Header("General Movement & Rotation")]
    // Holds the max speed and max acceleration of the character
    [SerializeField] float maxSpeed = 2.0f;
    [SerializeField] float maxAcceleration = 1.25f;

    // Holds the max angular acceleration and rotation of the character
    [SerializeField] float maxAngularAcceleration = 2.0f;
    [SerializeField] float maxRotation = 1.25f;

    // Holds the radi for arriving at the target and beginning to slow down
    [SerializeField] float targetRadius = 1.0f;
    [SerializeField] float slowRadius = 4.0f;

    // Holds the radi for arriving at the target and beginning to slow down
    [SerializeField] float targetAlignRadius = 1.0f;
    [SerializeField] float alignSlowRadius = 4.0f;

    [Header("Pursue")]
    // Holds the maximum prediction time (for pursue)
    [SerializeField] float maxPrediction = 5;

    [Header("Wander (old)")]
    // Holds the radius and forward offset of the wander circle (for wander old)
    [SerializeField] float wanderOffset = 3;
    [SerializeField] float wanderRadius = 3;
    // Holds the maximum rate at which the wander orientation can change
    [SerializeField] float wanderRate = 10;
    // Holds the current orientation of the wander target
    [SerializeField] float wanderOrientation = 0;

    [Header("Wander (new)")]
    // Wand radius, distance, jitter, and target position (for wander new)
    [SerializeField] float wanderRad = 5;
    [SerializeField] float wanderDistance = 5;
    [SerializeField] float wanderJitter = 1;
    Vector3 wanderTarget = Vector3.zero;

    [Header("Path Following")]
    // Holds the path to follow
    [SerializeField] Path path;
    // Holds the distance along the path to generate the target.
    // Can be negative if the character is to move along the reverse direction.
    float pathOffset = 0.5f;
    // Holds the curent position on the path
    [SerializeField] int currentParam;
    // Holds the time in the future to predict the character's position
    [SerializeField] float predictTime = 0.1f;
    // Arrival distance between path node and character
    float pathArrivalRadius = 5;
    // Which direction character is following the path
    [SerializeField] bool forwardPathTraversal = true;

    // Holds the time over which to achieve target speed
    [Header("Time To Target")]
    [SerializeField] float timeToTarget = 0.1f;
    [SerializeField] float alignmentTimeToTarget = 0.1f;

    [Header("Movement Visualization (HW 1)")]
    [SerializeField] GameObject targetPointPrefab;
    [SerializeField] GameObject targetPoint;
    [SerializeField] GameObject arriveRadiusPrefab;
    [SerializeField] GameObject arriveRadius;
    [SerializeField] GameObject pathNodes;
    [SerializeField] Text behaviorText;


    [Header("Ray Casting")]
    [SerializeField] float avoidDistance = 1.0f;
    [SerializeField] float lookAhead = 2.0f;


    private void Awake()
    {
        characterRb = this.GetComponent<Rigidbody2D>();
    }

    // Start is called before the first frame update
    void Start()
    {
        if (targetGO != null)
        {
            targetRb = targetGO.GetComponent<Rigidbody2D>();
        }
        prevMovement = movement;
    }

    void FixedUpdate()
    {
        if (prevMovement != movement && targetPoint != null)
        {
            prevMovement = movement;

            currentParam = 0;
            forwardPathTraversal = true;

            Destroy(targetPoint);
            DestroyArriveRadius();
            ShowPath(false);
        }

        if (targetGO == null || movement == MovementOperation.None)
        {
            WriteBehavior("None");
            return;
        }

        // Need to update character and target with their respective rigidbody values
        character = Rb2DToKinematic(characterRb);
        target = Rb2DToKinematic(targetRb);

        // Get SteeringOutput for current operation
        SteeringOutput steering;
        if (movement == MovementOperation.Seek)
        {
            steering = GetSeekSteering();
            WriteBehavior("SEEK");
        }
        else if (movement == MovementOperation.Flee)
        {
            steering = GetFleeSteering();
            WriteBehavior("FLEE");
        }
        else if (movement == MovementOperation.Arrive)
        {
            steering = GetArriveSteering();
            WriteBehavior("ARRIVE");
        }
        else if (movement == MovementOperation.Align)
        {
            steering = GetAlignSteering();
            WriteBehavior("ALIGN");
        }
        else if (movement == MovementOperation.VelocityMatching)
        {
            steering = GetVelocityMatchingSteering(target);
            WriteBehavior("VELOCITY MATCHING");
        }
        else if (movement == MovementOperation.Pursue)
        {
            steering = GetPursueSteering();
            WriteBehavior("DYNAMIC PURSUE (with dynamic arrive)");
        }
        else if (movement == MovementOperation.Evade)
        {
            steering = GetEvadeSteering();
            WriteBehavior("DYNAMIC EVADE");
        }
        else if (movement == MovementOperation.Face)
        {
            steering = GetFaceSteering();
            WriteBehavior("FACE");
        }
        else if (movement == MovementOperation.LookWhereYoureGoing)
        {
            steering = LookWhereYoureGoing();
            WriteBehavior("LOOK FORWARD");
        }
        else if (movement == MovementOperation.Wander)
        {
            steering = Wander();
            //steering = GetWanderSteering();
            WriteBehavior("WANDER");
        }
        else if (movement == MovementOperation.FollowPath)
        {
            ShowPath(true);
            //steering = GetPathSteering();
            steering = FollowPath();
            WriteBehavior("PATH FOLLOWING");
        }
        else if (movement == MovementOperation.RayCasting)
        {
            GetRayCastSteering();
            steering = GetSeekSteering();
            WriteBehavior("RAY CASTING");
        }
        else
        {
            steering = GetSeekSteering();
        }

        if (movement != MovementOperation.Align && 
            movement != MovementOperation.Face &&
            movement != MovementOperation.LookWhereYoureGoing)
        {
            // Get character to always face where it's going
            steering.angular = LookWhereYoureGoing().angular;
        }

        // Perform kinematic update
        character.Update(steering, maxSpeed, Time.fixedDeltaTime);

        // Write back to character RB at end of Update
        KinematicToRb2(ref characterRb, character);
    }

    #region Single Agent Movement (HW 1)
    // Seek
    // Returns the desired steering output
    SteeringOutput GetSeekSteering()
    {
        return GetSeekSteering(target);
    }

    SteeringOutput GetSeekSteering(Kinematic target)
    {
        // Create the structure to hold our output
        SteeringOutput steering = new SteeringOutput();

        // Get the direction to the target
        steering.linear = target.position - character.position;

        // Give full acceleration along this direction
        steering.linear.Normalize();
        steering.linear *= maxAcceleration;

        DrawTargetPoint(target.position);

        // Output the steering
        steering.angular = 0;
        return steering;
    }

    // Flee
    // Returns the desired steering output
    SteeringOutput GetFleeSteering()
    {
        return GetFleeSteering(target);
    }
    SteeringOutput GetFleeSteering(Kinematic target)
    {
        // Create the structure to hold our output
        SteeringOutput steering = new SteeringOutput();

        // Get the direction to the target
        steering.linear = character.position - target.position;

        // Give full acceleration along this direction
        steering.linear.Normalize();
        steering.linear *= maxAcceleration;

        DrawTargetPoint(target.position);

        // Output the steering
        steering.angular = 0;
        return steering;
    }

    // Arrive
    SteeringOutput GetArriveSteering()
    {
        return GetArriveSteering(target);
    }
    SteeringOutput GetArriveSteering(Kinematic target)
    {
        // Create the structure to hold our output
        SteeringOutput steering = new SteeringOutput() { linear = Vector3.zero, angular = 0 };

        // Get the direction to the target
        Vector3 direction = target.position - character.position;
        float distance = direction.magnitude;

        // Check if we are there, return no steering
        if (distance < targetRadius)
        {
            DrawArriveRadius(target.position, slowRadius);
            return steering;
        }

        // If we are outside the slowRadius, then go max speed
        // Otherwise, calculate a scaled speed
        float targetSpeed = 0.0f;
        if (distance > slowRadius)
        {
            DestroyArriveRadius();
            targetSpeed = maxSpeed;
        }
        else
        {
            DrawArriveRadius(target.position, slowRadius);
            targetSpeed = maxSpeed * distance / slowRadius;
        }

        // The target velocity combines speed and direction
        Vector3 targetVelocity = direction;
        targetVelocity.Normalize();
        targetVelocity *= targetSpeed;

        // Acceleration tries to get to the target velocity
        steering.linear = targetVelocity - character.velocity;
        steering.linear /= timeToTarget;

        // Check if the acceleration is too fast
        if (steering.linear.magnitude > maxAcceleration)
        {
            steering.linear.Normalize();
            steering.linear *= maxAcceleration;
        }

        DrawTargetPoint(target.position);

        // Output the steering
        return steering;
    }

    // Align
    SteeringOutput GetAlignSteering()
    {
        return GetAlignSteering(target);
    }
    SteeringOutput GetAlignSteering(Kinematic target)
    {
        // Create the structure to hold our output
        SteeringOutput steering = new SteeringOutput() { linear = Vector3.zero, angular = 0 };

        // Get the naive direction to the target
        float rotation = target.orientation - character.orientation;

        // Map the result to the (-pi, pi) interval
        rotation = MapToRange(rotation);
        float rotationSize = Mathf.Abs(rotation * Mathf.Rad2Deg);

        // Check if we are there, return no steering
        if (rotationSize < targetAlignRadius)
        {
            return steering;
        }

        // If we are outside the slowRadius, then use maximum rotation
        float targetRotation;
        if (rotationSize > alignSlowRadius)
        {
            targetRotation = maxRotation;
        }
        else
        {
            targetRotation = maxRotation * rotationSize / alignSlowRadius;
        }

        // The final target rotation combines speed and direction
        targetRotation *= rotation / rotationSize;

        // Acceleration tries to get to the target rotation
        steering.angular = targetRotation - character.rotation;
        steering.angular /= alignmentTimeToTarget;

        // Check if the acceleration is too great
        float angularAcceleration = Mathf.Abs(steering.angular);
        if (angularAcceleration > maxAcceleration)
        {
            steering.angular /= angularAcceleration;
            steering.angular *= maxAngularAcceleration;
        }

        // Output the steering
        return steering;
    }

    // Map given angle angle (in radians) to a range of (-pi,pi)
    private float MapToRange(float rotation)
    {
        float rad = rotation * Mathf.Deg2Rad;
        float pi2 = 2 * Mathf.PI;

        return rad - pi2 * Mathf.Floor((rad + Mathf.PI) / pi2);
    }

    // Velocity Matching
    SteeringOutput GetVelocityMatchingSteering(Kinematic target)
    {
        // Create the structure to hold out output
        SteeringOutput steering = new SteeringOutput() { linear = Vector3.zero, angular = 0 };

        // Acceleration tries to get to the target velocity
        steering.linear = target.velocity - character.velocity;
        steering.linear /= timeToTarget;

        // Check if the acceleration is too fast
        if (steering.linear.magnitude > maxAcceleration)
        {
            steering.linear.Normalize();
            steering.linear *= maxAcceleration;
        }

        // Output the steering
        return steering;
    }

    // Pursue
    SteeringOutput GetPursueSteering()
    {
        // 1. Calculate the target to delegate to seek
        //    Work out the distance to target
        Vector3 direction = target.position - character.position;
        float distance = direction.magnitude;

        //    Work out current speed
        float speed = character.velocity.magnitude;

        //    Check if speed is too small to give a reasonable prediction time
        //    Otherwise, calculate the prediction time
        float prediction;
        if (speed <= distance / maxPrediction)
        {
            prediction = maxPrediction;
        }
        else
        {
            prediction = distance / speed;
        }

        //    Put the target together
        Kinematic seekTarget = target;
        seekTarget.position += target.velocity * prediction;

        // 2. Delegate to seek
        return GetArriveSteering(seekTarget);
    }

    // Evade
    SteeringOutput GetEvadeSteering()
    {
        // 1. Calculate the target to delegate to seek
        //    Work out the distance to target
        Vector3 direction = target.position - character.position;
        float distance = direction.magnitude;

        //    Work out current speed
        float speed = character.velocity.magnitude;

        //    Check if speed is too small to give a reasonable prediction time
        //    Otherwise, calculate the prediction time
        float prediction;
        if (speed <= distance / maxPrediction)
        {
            prediction = maxPrediction;
        }
        else
        {
            prediction = distance / speed;
        }

        //    Put the target together
        Kinematic seekTarget = target;
        seekTarget.position += target.velocity * prediction;

        //DrawTargetPoint(seekTarget.position);

        // 2. Delegate to seek
        return GetFleeSteering(seekTarget);
    }

    // Face
    SteeringOutput GetFaceSteering()
    {
        // 1. Calculate the target to delegate to align
        //    Work out the direction to target
        Vector3 direction = target.position - character.position;

        //    Check for a zero direction, and make no change if so
        if (direction.magnitude == 0)
        {
            return new SteeringOutput { linear = Vector3.zero, angular = 0 };
        }

        //    Put the target together
        Kinematic alignTarget = target;
        alignTarget.orientation = Mathf.Atan2(-direction.x, direction.y) * Mathf.Rad2Deg;  // Book uses (-x,z); however, it's working in 3D

        // 2. Delegate to align
        return GetAlignSteering(alignTarget);
    }

    // Look in direction of movement
    SteeringOutput LookWhereYoureGoing()
    {
        // 1. Calculate the target to delegate to align
        //    Check for a zero direction, and make no change if so
        if (character.velocity.magnitude == 0)
        {
            return new SteeringOutput { linear = Vector3.zero, angular = 0 };
        }

        //    Otherwise, set the target based on the velocity
        target.orientation = Mathf.Atan2(-character.velocity.x, character.velocity.y) * Mathf.Rad2Deg;  // Book uses (-x,z); however, it's working in 3D

        // 2. Delegate to align
        return GetAlignSteering();
    }

    // Wander (old implementation)
    SteeringOutput GetWanderSteering()
    {
        // 1. Calculate the target to delegate to face
        //    Update the wander orientation
        float randomBinomial = RandomBinomial();
        wanderOrientation += (randomBinomial * wanderRate);

        //    Calculate the combined target orientation
        float targetOrientation = wanderOrientation + character.orientation;

        //    Calculate the center of the wander circle
        target.position = character.position + wanderOffset * scalarAsVector(character.orientation);

        //    Calculate the target location
        target.position += wanderRadius * scalarAsVector(targetOrientation);

        // 2. Delegate to face
        SteeringOutput steering = GetFaceSteering();

        // 3. Now set the linear acceleration to be at full acceleration in the direction of the orientation
        steering.linear = maxAcceleration * scalarAsVector(character.orientation);

        DrawTargetPoint(target.position);
        DrawArriveRadius(character.position, wanderRadius);

        // Return it
        return steering;
    }

    // Wander (current implementation)
    SteeringOutput Wander()
    {
        // Set wander target position
        wanderTarget += new Vector3(Random.Range(-1.0f, 1.0f) * wanderJitter,
                                    0,
                                    Random.Range(-1.0f, 1.0f) * wanderJitter);

        wanderTarget.Normalize();
        wanderTarget *= wanderRad;

        Vector3 targetLocal = wanderTarget + new Vector3(0, 0, wanderDistance);
        Vector3 targetWorld = this.gameObject.transform.InverseTransformVector(targetLocal);

        Kinematic wanderKinematicTarget = target;
        wanderKinematicTarget.position = targetWorld;

        DrawArriveRadius(character.position, wanderRad);

        // Seek wander target
        return GetSeekSteering(wanderKinematicTarget);
    }

    // Path Following (old implementation)
    SteeringOutput GetPathSteering()
    {
        // 1. Calculate the tagret to delegate to face
        //    Find the predicted future location
        Vector3 futurePos = character.position + character.velocity * predictTime;

        //    Find the current position on the path
        currentParam = path.GetParam(futurePos, currentParam);

        //    Offset it
        target.position = path.GetPosition(currentParam);

        DrawTargetPoint(target.position);

        // 2. Delegate to Seek
        return GetSeekSteering();
    }

    // Path Following (current implementation)
    SteeringOutput FollowPath()
    {
        // Check if the current node index is greater than the
        // the number of nodes in the path
        // Set index to path len - 1 and set it to traverse path in descending order
        if (currentParam >= path.path.Length)
        {
            currentParam = path.path.Length - 2;
            forwardPathTraversal = !forwardPathTraversal;
        }

        // Check if the current node index is negative
        // Reset index to 0 and set it to traverse path in ascending order
        if (currentParam < 0)
        {
            currentParam = 0;
            forwardPathTraversal = !forwardPathTraversal;
        }

        // Check if character is closer to another node further along in the path
        // If so, set the current node to that one
        if (forwardPathTraversal)
        {
            float distance = -1;
            for (int i = currentParam; i < path.path.Length; i++)
            {
                float dist = Vector3.Distance(character.position, path.path[i].position);
                if (distance < 0 || dist < distance)
                {
                    distance = dist;
                    currentParam = i;
                }
            }
        }
        else
        {
            float distance = -1;
            for (int i = currentParam; i >= 0; i--)
            {
                float dist = Vector3.Distance(character.position, path.path[i].position);
                if (distance < 0 || dist < distance)
                {
                    distance = dist;
                    currentParam = i;
                }
            }
        }

        target.position = path.path[currentParam].position;

        // If the distance between the character and target is less than
        // the path arrival radius
        // If so, set next node in path
        if (Vector3.Distance(character.position, target.position) < pathArrivalRadius)
        {
            if (forwardPathTraversal)
            {
                currentParam++;
            }
            else
            {
                currentParam--;
            }
        }
        // Arrive or seek to next node
        if (currentParam == 0 || currentParam == path.path.Length - 1)
        {
            return GetArriveSteering();
        }
        DestroyArriveRadius();
        return GetSeekSteering();
    }
    #endregion


    // Ray Casting
    void GetRayCastSteering()
    {
        //  1. Calculate the target to delegate to seek
        //     Determine collision
        Vector2 pos = character.position;
        Vector2 dir = target.position - character.position;

        RaycastHit2D hit = Physics2D.Raycast(pos, dir, lookAhead);
        
        if (hit.transform != null)
        {
            Debug.DrawRay(pos, dir, Color.green);
        }
        else
        {
            Debug.DrawRay(pos, dir, Color.red);
        }
    }




    #region Visualize Single Agent Movement (HW 1)
    // Generate a random binomial
    float RandomBinomial()
    {
        return Random.Range(0.0f, 1.0f) - Random.Range(0.0f, 1.0f);
    }

    // Transform a scalar to a vector (used for orientaiton)
    public static Vector3 scalarAsVector(float w)
    {
        return new Vector3(Mathf.Sin(w) * Mathf.Rad2Deg, Mathf.Cos(w) * Mathf.Rad2Deg);
    }

    // Draw the target point that the character will move to/from
    void DrawTargetPoint(Vector3 position)
    {
        // Draw Target Point
        if (targetPointPrefab != null)
        {
            if (targetPoint == null)
            {
                // Display a dot at target position
                targetPoint = Instantiate(targetPointPrefab);
            }
            targetPoint.transform.position = position;
        }
    }

    // Visualize the arrive radius around a given point
    void DrawArriveRadius(Vector3 position, float radius)
    {
        if (arriveRadius == null && arriveRadiusPrefab != null)
        {
            arriveRadius = Instantiate(arriveRadiusPrefab);
        }

        arriveRadius.transform.localScale = new Vector3(radius, radius, 1);
        arriveRadius.transform.position = position;
    }

    // Destroy the arrive radius visualization if it exists
    void DestroyArriveRadius()
    {
        if (arriveRadius != null)
        {
            Destroy(arriveRadius);
        }
    }

    // Display path for character to follow
    void ShowPath(bool show)
    {
        if (pathNodes == null)
            return;
        pathNodes.SetActive(show);
    }

    // Display the name of the given behavior
    void WriteBehavior(string behavior)
    {
        if (behaviorText != null)
            behaviorText.text = behavior;
    }
    
    // Convert a Rigidbody2D to a Kinematic
    Kinematic Rb2DToKinematic(Rigidbody2D rb)
    {
        Kinematic kinematic = new Kinematic();
        kinematic.position = rb.position;
        kinematic.orientation = rb.rotation;
        kinematic.velocity = rb.velocity;
        kinematic.rotation = rb.angularVelocity;
        return kinematic;
    }

    // Convert a Kinematic to a Rigidbody2D
    void KinematicToRb2(ref Rigidbody2D rb, Kinematic kinematic)
    {
        rb.position = kinematic.position;
        rb.rotation = kinematic.orientation;
        rb.velocity = kinematic.velocity;
        rb.angularVelocity = kinematic.rotation;
    }
    #endregion

    public struct Kinematic
    {
        public Vector3 position;   // Location in the world a character exists     (Rigidbody2D.position)
        public float orientation;  // Direction in which a character is facing     (Rigidbody2D.rotation)

        public Vector3 velocity;   // Linear Velocity                              (Rigidbody2D.velocity)
        public float rotation;     // Angular Velocity                             (Rigidbody2D.angularVelocity)

        public void Update(SteeringOutput steering, float maxSpeed, float time)
        {
            if (steering.angular != 0)
            position += velocity * time;
            orientation += rotation * time;

            // and the velocity and rotation
            velocity += steering.linear * time;
            orientation += steering.angular * time;

            // Check for speeing and clip
            if (velocity.magnitude > maxSpeed)
            {
                velocity.Normalize();
                velocity *= maxSpeed;
            }
        }
    }

    public struct SteeringOutput
    {
        public Vector3 linear;     // Linear (movement) Acceleration
        public float angular;      // Angular (rotational) Acceleration
    }

    [System.Serializable]
    public struct Path
    {
        public Transform[] path;

        public int GetParam(Vector3 position, int lastParam)
        {
            int nextPosition = -1;
            float nextDistance = -1.0f;

            //lastParam %= path.Length;
            if (lastParam >= path.Length - 1)   // Reached the end of the path
            {
                for (int i = path.Length - 1; i >= 0; i--)
                {
                    float distance = (position - path[i].position).magnitude;
                    if (nextDistance < 0.0f || distance < nextDistance)
                    {
                        nextDistance = distance;
                        nextPosition = i;
                    }
                }
            }
            else
            {
                for (int i = lastParam; i < path.Length; i++)
                {
                    float distance = (position - path[i].position).magnitude;
                    if (nextDistance < 0.0f || distance < nextDistance)
                    {
                        nextDistance = distance;
                        nextPosition = i;
                    }
                }
            }
            return nextPosition;
        }

        public Vector3 GetPosition(int param)
        {
            param %= path.Length;

            return path[param].position;
        }
    }

    public enum MovementOperation
    {
        None,

        // Standard Movement
        Seek,
        Flee,
        Arrive,
        Align,
        VelocityMatching,
        Pursue,
        Evade,
        Face,
        LookWhereYoureGoing,
        Wander,
        FollowPath, 

        // Movement with Obstacle Avoidance
        RayCasting
    }
}
