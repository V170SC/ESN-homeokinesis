# ESN-homeokinesis
Dissertation on homeokinesis and Echo State Network control

Some experiments performed for my dissertation to get an MsC. in Artificial Inteligence.

Directed exploration in reinforcement learning
Reinforcement learning in a high-dimensional task, such as skill learning in a humanoid robot, can be extremely time consuming if done through the classical methods. For this project I used a self-organizing homeokinetic controller together with the idea of conceptors to direct the exploration of the behavioral space in order to learn useful skills in a reasonable amount of time with relatively few trials. For this approach I trained a Recurrent Neural Network (using an Echo-State approach) with different kinds of movements of a ball robot (a hexapod and a humanoid robot were also used with varied results) derived from the homeokinetic controller. Each different movement was stored in the Networks memory (represented by a matrix) and then reproduced using the conceptor (represented as a vector) as a filter. These movements were then supplied as actions for the robot to take in a reinforcement learning problem. Supervised by Dr. Michael Herrmann.

The whole project was done in 10 weeks, the scope had to be 

You can find the whole document in the dissertation folder.
Some sample Matlab code done for testing the Echo State Networks and the Conceptors is in the ESNConceptorsTest folder.
The lpzRobots folder contains a build for the simulator used.
The Pictures folder contains some sample pictures of the experiments performed.
ReferencePapers has the main papers cited for this work.
Finally SimlationExperiments is the code that runs the final experiments for a Spherical Robot being controlled with an Echo State Network controller and a homeokinetic approach.
