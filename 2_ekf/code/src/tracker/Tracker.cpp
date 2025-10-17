#include "tracker/Tracker.h"

#define EUCLIDEAN

Tracker::Tracker()
{
    cur_id_ = 0;
    distance_threshold_ = 0.40; // meters
    covariance_threshold = 50.0; 
    loss_threshold = 30; //number of frames the track has not been associated with anyone
}
Tracker::~Tracker()
{
}

/*
    This function removes tracks based on any strategy
*/
void Tracker::removeTracks()
{
    std::vector<Tracklet> tracks_to_keep;

    for (size_t i = 0; i < tracks_.size(); ++i) // iterate over all the tracklets 
    {
        // TODO
        // Implement logic to discard old tracklets

        if ((tracks_[i].getLossCount() < loss_threshold) && (tracks_[i].getXCovariance() < covariance_threshold)  && (tracks_[i].getYCovariance() < covariance_threshold))
        {
           tracks_to_keep.push_back(tracks_[i]);
        }
    }

    tracks_.swap(tracks_to_keep);
}

/*
    This function add new tracks to the set of tracks ("tracks_" is the object that contains this)
*/
void Tracker::addTracks(const std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{
    // Adding not associated detections
    for (size_t i = 0; i < associated_detections.size(); ++i)
        if (!associated_detections[i])
            tracks_.push_back(Tracklet(cur_id_++, centroids_x[i], centroids_y[i]));
}

static inline double euclidean_distance(Tracklet& t, double x, double y)
{
    return std::hypot(t.getX() - x, t.getY() - y);
}

/*
    This function associates detections (centroids_x,centroids_y) with the tracks (tracks_)
    Input:
        associated_detection an empty vector to host the associated detection
        centroids_x & centroids_y measurements representing the detected objects
*/
void Tracker::dataAssociation(std::vector<bool> &associated_detections, const std::vector<double> &centroids_x, const std::vector<double> &centroids_y)
{

    //Remind this vector contains a pair of <track_idx, detection_idx>
    associated_track_det_ids_.clear();
    size_t number_of_detections = associated_detections.size();
    size_t number_of_tracks = tracks_.size(); 

    for (size_t i = 0; i < number_of_tracks; ++i) // for each track find the correspondent detection using a distance metric (euclidean, mahalanobis, ...)
    {
        int closest_point_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < number_of_detections; ++j)
        {
            // TODO
            // Implement logic to find the closest detection (centroids_x,centroids_y) 
            // to the current track (tracks_) 
            double current_distance = std::numeric_limits<double>::max();
            #ifdef EUCLIDEAN
                current_distance = euclidean_distance(tracks_[i], centroids_x[j], centroids_y[j]);
            #elif defined(MAHALANOBIS)
                current_distance = mahalanobis_distance(tracks_[i], centroids_x[j], centroids_y[i]);
            #else 
                #error "No distance metric defined at compile time"
            #endif

            if (current_distance < min_dist)
            {
                min_dist = current_distance;
                closest_point_id = j;
            }
        }

        // Associate the closest detection to a tracklet
        if (min_dist < distance_threshold_ && !associated_detections[closest_point_id])
        {
            associated_track_det_ids_.push_back(std::make_pair(i, closest_point_id)); // i = tracklet idx, closest_point_id = detection idx
            associated_detections[closest_point_id] = true;
        }
    }
    // at the end of this block I could have detections not associated to any tracklet and tracklets not associated to any detection
}

void Tracker::track(const std::vector<double> &centroids_x,
                    const std::vector<double> &centroids_y,
                    bool lidarStatus)
{

    std::vector<bool> associated_detections(centroids_x.size(), false); 
    // associated_detections is just to keep track of the detections that we have associated to a tracklet after the association process.
    // the elements of associated_detections that are false after the association process 

    // TODO: Predict the position
    //For each track --> Predict the position of the tracklets
    for (auto& track : tracks_)
    {
        track.predict();
    }
    
    // TODO: Associate the predictions with the detections
    dataAssociation(associated_detections, centroids_x, centroids_y);

    // Update tracklets with the new detections
    for (int i = 0; i < associated_track_det_ids_.size(); ++i)
    {
        auto track_id = associated_track_det_ids_[i].first;
        auto det_id = associated_track_det_ids_[i].second;

        // if the track_id has recieved a detection than we reset the no_detection_counter to zero
        tracks_[track_id].update(centroids_x[det_id], centroids_y[det_id], lidarStatus);
    }

    // TODO: Remove dead tracklets
    removeTracks();

    // TODO: Add new tracklets
    addTracks(associated_detections, centroids_x, centroids_y);
}
