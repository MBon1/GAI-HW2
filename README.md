# GAI-HW2
Homework (Project) II: Obstacle Avoidance 

Student: Matthew Bonnecaze

I developed a demonstration of the following obstacle avoidance algorithms: 
 - Ray Casting
 - Cone Check
 - Collision Prediction

The demonstration has the player, which can be rotated (clockwise or counterclockwise) and moved (forwards or backwards), while an agent moves autonomously, using the player as the target.  The prey (green) character pursues the player and mainly uses a cone check for obstacle avoidance. The preditor (red) character pursues the prey and mainly uses ray casting. This was done to show the differences in each algorithms' behavior. In the middle of the screen is a large wall to test on, and to  the right is a field of moving obstacles.  In the field of moving obstacles, the preditor and prey will switch to using collision prediction to evade the moving obstacles.

It is important to note that some characters have a list of ignored game objects that, even if detected for obstacle avoidance, it will be ignored. 

For each algorithm, visual cues are given to demonstrate how they operate: 
 - Ray Casting
	- Green lines are ray casts that did not collide into anything
	- Red lines are ray casts that collided into something
 - Cone Check
	- Grey lines outline the cone
 - Collision Prediction
	- Yellow lines connect the character's current position and the target that would collide soonest's predicted position


Demo Control Scheme: 
 - Move Player Character
    - Forwards (W / Up-Arrow)
    - Backwards (S / Down-Arrow)

 - Rotate Player Character
    - Counterclockwise (A / Left-Arrow)
    - Clockwise (D / Right-Arrow)

 - Return to Title Scene (Esc)



Major Files: 
 - Movement_3.cs	(contains the functions to perform agent movement)
 - Ray Casting : GetRayCastSteering()
 - Cone Check : GetConeCheckSteering()
 - Collision Prediction : CollisionPrediction()

My implementation of ray casting uses Unity's ray cast system and projects 3 rays. This allows it to navigate around large objects like the wall easily; however, it stuggles to find smaller objects.

My implementation of cone cast finds all objects that are marked to be targeted (not in ignore list) that are within the cone. This works for smaller targets (about the size of the character); however, larger objects like the wall can lead to the character getting stuck.

My implementation of collision prediction finds the object that is not in ignore list that is predicted to collide with the character. It then uses this predicted position to evade the
target.

In my implementation, I have the obstacle avoidance algorithms be called first, and if they needed to perform some course correction to prevent collision, then no other movement behvaior is called; however, if the algorithm does not need to change course, the movement behavior is called.

Plenty of holes in the demo area's design and some of the implementation was caused due to time constraints (homework due every day this past week), leaving me with only a day to work on this assignment.  While this is not really necessary, I figured it was worth providing an explanation as to the quality of my work.