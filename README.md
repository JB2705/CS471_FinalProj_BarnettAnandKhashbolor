The uploaded file implements the core A star algorithm, which in essence finds the least cost path through a weighted graph by traversing the nodes with the lowest total accumulated path cost (g) added to the heuristic of the estimated remaining cost to the goal (h).
It is optimal when the heuristic used is admissible, meaning it never over-estimates the remaining cost to the goal. It includes 4 test cases, including edge cases, from different graphs, which show the expected/optimal answer, and the computed answer. 
The algorithm passed all of the given test cases. These test cases automatically run upon running the python file. More information about the program can also be found in the comments of the code. 
ChatGPT was used to create the core code in the program, along with the test cases which were verified by us. 
There is lots more work to be done, mainly we need to implement a system that actually computes the heuristics for each graph instead of them being given, which is the current system.
Additionally, we need to flesh out how we are going to handle dynamic obstacle costs within the algorithm and what costs we will associate with different types of hazards and terrain. 
