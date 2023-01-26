Homework (Project) I: Single Agent Movement

Student: Matthew Bonnecaze

I developed a demonstration of the following behavior algorithms: 
 - Dynamic Pursue with Dynamic Arrive
 - Dynamic Evade
 - Dynamic Wander
 - Path Following

I also included a way to demonstrate: 
 - Dynamic Seek
 - Dynamic Flee

The demonstration has the player, which can be rotated (clockwise or counterclockwise) and moved (forwards or backwards), while an agent moves autonomously, using the player as the target.  The player can freely switch between which algorithm is being used by pressing a corresponding numeric key (0-6), see "Setting NPC Algorithm" under "Demo Control Scheme".  

For each algorithm, visual cues are given to demonstrate how they operate.  For example, the slow radius is displayed when arriving and the target’s location (predicted for pursue/evade and actual for seek/flee) is shown.  


Demo Control Scheme: 
 - Move Player Character
    - Forwards (W / Up-Arrow)
    - Backwards (S / Down-Arrow)

 - Rotate Player Character
    - Counterclockwise (A / Left-Arrow)
    - Clockwise (D / Right-Arrow)

 - Return to Title Scene (Esc)

 - Setting NPC Algorithm: 
    - None (0)
    - Seek (1)
    - Flee (2)
    - Pursue (3)
    - Evade (4)
    - Wander (5)
    - Follow Path (6)


Major Files: 
 - Movement_3.cs	(contains the functions to perform agent movement)
   - Dynamic Seek: GetSeekSteering()
   - Dynamic Flee: GetFleeSteering()
   - Dynamic Pursue: GetPursueSteering()
   - Dynamic Evade: GetEvadeSteering()
   - Dynamic Wander: Wander()
   - Follow Path: FollowPath()
   - Kinematic Struct Update: Kinematic/Update()

My implementation of Dynamic Seek targets the player and will follow the targeted player.  
My implementation of Dynamic Flee targets the player and will move away from the targeted player.  
My implementation of Dynamic Pursue predicts where the Player will be and tries to target the player's predicted position and will move towards the predicted position.  At present, however, there is minor oscillation once the character reaches the player.  
My implementation of Dynamic Evade predicts where the Player will be and tries to avoid the player's predicted position and will move away from it.  
My implementation of Dynamic Wander determines a point to move towards at random intervals. At present, the implementation does seem to sometimes set a target outside of the wander radius.  
My implementation of Path Following takes a path and will travel up and down a given path.  While being set to start from the first point in the path, I allow for the character to skip to a closer point further in the path than its current target if this new point is closer than its present targeted point.  Once the last node in the path is reached, the character will switch its direction and move back down the path in the opposite direction it came from.  
