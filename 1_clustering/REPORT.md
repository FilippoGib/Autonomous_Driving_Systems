Academic Year: 2025/2026

# Report on Clustering assignment development

dataset_1
-
On dataset_1 the code works well at high fps. Using RANSAC we are able to remove all the planar components and we are left we small pointclouds, of around all the same size and shape, well disconnected from one another. \
Given this conditions it is easier to tune the parameters:
- __setClusterTolerance__ offers a bigger interval where all the points of the obstacle are included in the cluster without merging multiple clusters

- __minClusterSize__ and __maxClusterSize__ must model instances of the same distribution (all the clusters that we want to extract have more or less the same number of points)

dataset_2
-
Working on dataset_2 is a greater challenge because we lose some of the benefits of dataset_1:

1. the scene has some perceptual noise that we cannot remove using the planar model: tree, bushes, hydrants. Furthermore, these elements have a wierd shape and therefore changing the clusterTolerance to filter them is not a reliable approach because instead of clustering the whole tree, we cluster the branches, separately which is still something that we want to avoid.

1. the clusters that we want to model do not belong to the same distribution:
  - Pedestrians
  - Bikes
  - Cars

Given these two major problems, one possible solution would be to keep the clustering parameters loose at first, in order to filter out most of the noise (ground, walls, sidewalks, etc.) and subsequentially pass the clusters to a function that uses heuristics to discriminate between our classes of interest: pedestrian, bike, car. 

This method can achieve good results, expecially if we use well though heuristics like ratios between the sides, ratios of the diagnals, ratios of the faces of the 3Dbb. 

However the big drawback is that we need to manually tune the parameters of this classifier, which whould be acceptable if we could find at least some parameters that work on a big dataset, but in this case we only have one specific sequence therefore finetuning our algorithm would surely lead to overfitting -> Almost certaintly the paramters that achieve a perfect result on this specific dataset will not work first-shot on another (similar) scene.

An alternative solution would be to adopt an automatic learining model eg: pointpillars (maybe even pretrained) to do all the work for us.




