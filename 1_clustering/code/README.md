Academic Year: 2025/2026

* Assignment #1 Euclidean clustering object detection:
    - Objective: Find and segment cars and pedestrians lying on the road
        + 1) Implement an euclidean clustering program using the PCL library as studied during the lectures. Please follow the instructions and comments detailed in the code (cluster_extraction.cpp)
        + 2) Optional assignment: Implement the algorithm that performs the euclidean clustering (use the proximity function and follow the pseudo-code), plots the distance of each cluster, color nearby front vehicles and stress your approach on dataset 2.
        + 3) Try to make your solution work on dataset 2. If you cannot successfully provide a solution. Write a short report explaining why.
       
    - Tasks evaluated (evaluated over 15 points):
        + The code compiles and works (2 points)
        + Good cluster detection without false positives and high frame rate (5 points)
        + Optional clustering implementation (1 point)
        + Solution working on dataset 2 or report provided explaining how to make it work (1 point)
        + Extend the code to support any additional useful functionality (4 points). Examples:
			* Plot the distance of each cluster w.r.t ego vehicle 
			* Color in red the vehicles that are both in front of and 5 meters away from the ego vehicle 
			* Any other functionality is up to your discretion (use other strategies to create the clustering rather than the Euclidean heuristic)

* OS requirements:
    -A Linux based operating system or virtual machine (Ubuntu 18-24 is fine)
    -PCL LIBRARY (sudo apt-get install pcl-tools | sudo apt install libpcl-dev)
    -cmake (sudo apt-get install cmake)

* Instructions to compile the code:
    cd build
    cmake ..
    make
    ./cluster_extraction

* IMPORTANT:
You have to change the following line of code:
  std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/media/nacho/DATA/Sensors/pcl/dataset_1/"},
                                             boost::filesystem::directory_iterator{});

into the path that contains your dataset 
or modify the code to accept a command line dataset
