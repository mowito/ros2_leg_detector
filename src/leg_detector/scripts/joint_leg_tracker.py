#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import sys
import signal



# Custom messages
from leg_detector_msgs.msg import Person, PersonArray, Leg, LegArray

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Standard Python modules
import numpy as np
import random
import math
from scipy.optimize import linear_sum_assignment
import scipy.stats
import scipy.spatial
from geometry_msgs.msg import PointStamped, Point, Quaternion
import tf2_ros
import tf2_py
import tf2_geometry_msgs
import tf2_kdl
import copy
import timeit
import message_filters
import sys

# external packages
from pykalman import KalmanFilter


signal.signal(signal.SIGINT, lambda x, y: sys.exit(0))

"""
A detected scan cluster. Not yet associated to an existing track.
"""
class DetectedCluster:
    """
    Constructor
    """
    def __init__(self, pos_x, pos_y, confidence, in_free_space):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.confidence = confidence
        self.in_free_space = in_free_space
        self.in_free_space_bool = None

"""
A tracked object. Could be a person leg, entire person or any arbitrary object in the laser scan.
"""
class ObjectTracked:

    new_leg_id_num = 1

    """
    Constructor
    """
    def __init__(self, x, y, now, confidence, is_person, in_free_space):
        self.id_num = ObjectTracked.new_leg_id_num
        ObjectTracked.new_leg_id_num += 1
        self.colour = (random.random(), random.random(), random.random())
        self.last_seen = now
        self.seen_in_current_scan = True
        self.times_seen = 1
        self.confidence = confidence
        self.dist_travelled = 0.
        self.is_person = is_person
        self.deleted = False
        self.in_free_space = in_free_space

        # People are tracked via a constant-velocity Kalman filter with a Gaussian acceleration distrubtion
        # Kalman filter params were found by hand-tuning. 
        # A better method would be to use data-driven EM find the params. 
        # The important part is that the observations are "weighted" higher than the motion model 
        # because they're more trustworthy and the motion model kinda sucks

        #scan_frequency = KalmanMultiTrackerNode.get_parameter_or("scan_frequency", 7.5)
        scan_frequency = 7.5
        
        delta_t = 1./scan_frequency
        delta_t = 1./scan_frequency
        if scan_frequency > 7.49 and scan_frequency < 7.51:
            std_process_noise = 0.06666
        elif scan_frequency > 9.99 and scan_frequency < 10.01:
            std_process_noise = 0.05
        elif scan_frequency > 14.99 and scan_frequency < 15.01:
            std_process_noise = 0.03333
        else:
            print ("Scan frequency needs to be either 7.5, 10 or 15 or the standard deviation of the process noise needs to be tuned to your scanner frequency")
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1
        var_pos = std_pos**2
        var_vel = std_vel**2
        # The observation noise is assumed to be different when updating the Kalman filter than when doing data association
        var_obs_local = std_obs**2 
        self.var_obs = (std_obs + 0.4)**2

        self.filtered_state_means = np.array([x, y, 0, 0])
        self.pos_x = x
        self.pos_y = y
        self.vel_x = 0
        self.vel_y = 0

        self.filtered_state_covariances = 0.5*np.eye(4) 

        # Constant velocity motion model
        transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])

        # Oberservation model. Can observe pos_x and pos_y (unless person is occluded). 
        observation_matrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]])

        transition_covariance = np.array([[var_pos,       0,       0,       0],
                                          [      0, var_pos,       0,       0],
                                          [      0,       0, var_vel,       0],
                                          [      0,       0,       0, var_vel]])

        observation_covariance =  var_obs_local*np.eye(2)

        self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )


    """
    Update our tracked object with new observations
    """
    def update(self, observations):

        self.filtered_state_means, self.filtered_state_covariances = (
            self.kf.filter_update (
                self.filtered_state_means,
                self.filtered_state_covariances,
                observations
            )
        )

        # Keep track of the distance it's travelled 
        # We include an "if" structure to exclude small distance changes, 
        # which are likely to have been caused by changes in observation angle
        # or other similar factors, and not due to the object actually moving
        delta_dist_travelled = ((self.pos_x - self.filtered_state_means[0])**2 + (self.pos_y - self.filtered_state_means[1])**2)**(1./2.)
        if delta_dist_travelled > 0.01: 
            self.dist_travelled += delta_dist_travelled


        self.pos_x = self.filtered_state_means[0]
        self.pos_y = self.filtered_state_means[1]
        self.vel_x = self.filtered_state_means[2]
        self.vel_y = self.filtered_state_means[3]

"""
Tracker for tracking all the people and objects
"""
class KalmanMultiTrackerNode(Node):

    max_cost = 9999999

    """
    Constructor
    """
    def __init__(self):
        super().__init__("KalmanMultiTracker")
    
        self.objects_tracked = []
        self.potential_leg_pairs = set()
        self.potential_leg_pair_initial_dist_travelled = {}
        self.people_tracked = []
        self.prev_track_marker_id = 0
        self.prev_person_marker_id = 0
        self.prev_time = None
        self.buffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.buffer, self)
        self.local_map = None
        self.new_local_map_received = True
        random.seed(1)
        
        # Get ROS params
        self.fixed_frame = self.get_parameter_or("fixed_frame","laser")
        self.max_leg_pairing_dist = self.get_parameter_or("max_leg_pairing_dist", 0.8)
        self.confidence_threshold_to_maintain_track = self.get_parameter_or("confidence_threshold_to_maintain_track", 0.1)
        self.publish_occluded = self.get_parameter_or("publish_occluded", True)
        self.publish_people_frame = self.get_parameter_or("publish_people_frame", self.fixed_frame)
        self.use_scan_header_stamp_for_tfs = self.get_parameter_or("use_scan_header_stamp_for_tfs", False)
        self.publish_detected_people = self.get_parameter_or("display_detected_people", False)
        self.dist_travelled_together_to_initiate_leg_pair = self.get_parameter_or("dist_travelled_together_to_initiate_leg_pair", 0.5)
        scan_topic = self.get_parameter_or("scan_topic", "/scan")
        self.scan_frequency = self.get_parameter_or("scan_frequency", 7.5)
        self.in_free_space_threshold = self.get_parameter_or("in_free_space_threshold", 0.06)
        self.confidence_percentile = self.get_parameter_or("confidence_percentile", 0.90)
        self.max_std = self.get_parameter_or("max_std", 0.9)

        self.mahalanobis_dist_gate = scipy.stats.norm.ppf (1.0 - (1.0-self.confidence_percentile)/2., 0, 1.0)
        self.max_cov = self.max_std**2
        self.latest_scan_header_stamp_with_tf_available = self.get_clock().now()
        


    	# ROS publishers
        self.people_tracked_pub = self.create_publisher(PersonArray, "people_tracked", 300)
        self.people_detected_pub = self.create_publisher(PersonArray, "people_detected", 300)
        self.marker_pub = self.create_publisher(Marker, "visualization_marker", 300)
        self.non_leg_clusters_pub = self.create_publisher(LegArray, "non_leg_clusters", 300)

        # ROS subscribers 
        self.detected_clusters_sub = self.create_subscription(LegArray, "detected_leg_clusters", self.detected_clusters_callback, 300)
        self.local_map_sub = self.create_subscription(OccupancyGrid, "local_map", self.local_map_callback, 300)

        rclpy.spin(self)


    def local_map_callback(self, map):

        self.local_map = map
        self.new_local_map_received = True
        

    """
    Determine the degree to which the position (x,y) is in freespace according to our local map
    @rtype:     float
    @return:    degree to which the position (x,y) is in freespace (range: 0.-1.)
    """
    def how_much_in_free_space(self, x, y):

        # If we haven't got the local map yet, assume nothing's in freespace
        if self.local_map == None:
            return self.in_free_space_threshold*2

        # Get the position of (x,y) in local map coords
        map_x = int(round((x - self.local_map.info.origin.position.x)/self.local_map.info.resolution))
        map_y = int(round((y - self.local_map.info.origin.position.y)/self.local_map.info.resolution))

        # Take the average of the local map's values centred at (map_x, map_y), with a kernal size of <kernel_size>
        # If called repeatedly on the same local_map, this could be sped up with a sum-table
        sum = 0
        kernel_size = 2
        for i in range(map_x-kernel_size, map_x+kernel_size):
            for j in range(map_y-kernel_size, map_y+kernel_size):
                if i + j*self.local_map.info.height < len(self.local_map.data):
                    sum += self.local_map.data[i + j*self.local_map.info.height]
                else:  
                    # We went off the map! position must be really close to an edge of local_map
                    return self.in_free_space_threshold*2
        percent = sum/(((2.*kernel_size + 1)**2.)*100.)
        
        return percent


    """
    Match detected objects to existing object tracks using a global nearest neighbour data association
    """    
    def match_detections_to_tracks_GNN(self, objects_tracked, objects_detected):
        
        matched_tracks = {}

        # Populate match_dist matrix of mahalanobis_dist between every detection and every track
        match_dist = [] # matrix of probability of matching between all people and all detections.   
        eligable_detections = [] # Only include detections in match_dist matrix if they're in range of at least one track to speed up munkres
        for detect in objects_detected:
            at_least_one_track_in_range = False
            new_row = []
            for track in objects_tracked:
                # Ignore possible matchings between people and detections not in freespace 
                if track.is_person and not detect.in_free_space_bool:
                    cost = self.max_cost 
                else:
                    # Use mahalanobis dist to do matching
                    cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                    mahalanobis_dist = math.sqrt(((detect.pos_x-track.pos_x)**2 + (detect.pos_y-track.pos_y)**2)/cov) # = scipy.spatial.distance.mahalanobis(u,v,inv_cov)**2
                    if mahalanobis_dist < self.mahalanobis_dist_gate:
                        cost = mahalanobis_dist
                        at_least_one_track_in_range = True
                    else:
                        cost = self.max_cost
                new_row.append(cost)
            # If the detection is within range of at least one track, add it as an eligable detection in the munkres matching 
            if at_least_one_track_in_range: 
                match_dist.append(new_row)
                eligable_detections.append(detect)

        # Run munkres on match_dist to get the lowest cost assignment
        if match_dist:
            elig_detect_indexes, track_indexes = linear_sum_assignment(match_dist)
            for elig_detect_idx, track_idx in zip(elig_detect_indexes, track_indexes):
                if match_dist[elig_detect_idx][track_idx] < self.mahalanobis_dist_gate:
                    detect = eligable_detections[elig_detect_idx]
                    track = objects_tracked[track_idx]
                    matched_tracks[track] = detect

        return matched_tracks

    
    """
    Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
    It will try to match the newly detected clusters with tracked clusters from previous frames.
    """
    def detected_clusters_callback(self, detected_clusters_msg):

        # Waiting for the local map to be published before proceeding. This is ONLY needed so the benchmarks are consistent every iteration
        # Should be removed under regular operation
        if self.use_scan_header_stamp_for_tfs:       # Assume <self.use_scan_header_stamp_for_tfs> means we're running the timing benchmark
            wait_iters = 0
            while self.new_local_map_received == False and wait_iters < 10:
                #insert sleep command
                wait_iters += 1
            if wait_iters >= 10:
                self.get_logger().info("no new local_map received. Continuing anyways")
            else:
                self.new_local_map_received = False
                
        now = detected_clusters_msg.header.stamp
        
        detected_clusters = []
        detected_clusters_set = set()
        for cluster in detected_clusters_msg.legs:
            new_detected_cluster = DetectedCluster(
                cluster.position.x,
                cluster.position.y,
                cluster.confidence,
                in_free_space = self.how_much_in_free_space(cluster.position.x, cluster.position.y)
            )
            if new_detected_cluster.in_free_space < self.in_free_space_threshold:
                new_detected_cluster.in_free_space_bool = True
            else:
                new_detected_cluster.in_free_space_bool = False
            
            detected_clusters.append(new_detected_cluster)
            detected_clusters_set.add(new_detected_cluster)

            # Propogate existing tracks
        
        to_duplicate = set()
        propogated = copy.deepcopy(self.objects_tracked)
        for propogated_track in propogated:
            propogated_track.update(np.ma.masked_array(np.array([0, 0]), mask=[1,1]))
            if propogated_track.is_person:
                to_duplicate.add(propogated_track)
        
        # Duplicate tracks of people so they can be matched twice in the matching
        duplicates = {}
        for propogated_track in to_duplicate:
            propogated.append(copy.deepcopy(propogated_track))
            duplicates[propogated_track] = propogated[-1]

        # Match detected objects to existing tracks
        matched_tracks = self.match_detections_to_tracks_GNN(propogated, detected_clusters)

        # Publish non-human clusters so the local grid occupancy map knows which scan clusters correspond to people
        non_legs_msg = LegArray()
        non_legs_msg.header = detected_clusters_msg.header
        leg_clusters = set()
        for track, detect in matched_tracks.items():
            if track.is_person:
                leg_clusters.add(detect)
        non_leg_clusters = detected_clusters_set.difference(leg_clusters)
        for detect in non_leg_clusters:
            #non_leg = Leg(Point(detect.pos_x, detect.pos_y, 0), 1)
            non_leg = Leg()
            non_leg.position.x = detect.pos_x
            non_leg.position.y = detect.pos_y
            non_leg.position.z = 0.0
            non_leg.confidence = 1.0
            non_legs_msg.legs.append(non_leg)
        self.non_leg_clusters_pub.publish(non_legs_msg)

        # Update all tracks with new oberservations 
        tracks_to_delete = set()
        for idx, track in enumerate(self.objects_tracked):
            propogated_track = propogated[idx] # Get the corresponding propogated track
            if propogated_track.is_person:
                if propogated_track in matched_tracks and duplicates[propogated_track] in matched_tracks:
                    # Two matched legs for this person. Create a new detected cluster which is the average of the two
                    md_1 = matched_tracks[propogated_track]
                    md_2 = matched_tracks[duplicates[propogated_track]]
                    matched_detection = DetectedCluster((md_1.pos_x+md_2.pos_x)/2., (md_1.pos_y+md_2.pos_y)/2., (md_1.confidence+md_2.confidence)/2., (md_1.in_free_space+md_2.in_free_space)/2.)
                elif propogated_track in matched_tracks:
                    # Only one matched leg for this person
                    md_1 = matched_tracks[propogated_track]
                    md_2 = duplicates[propogated_track]
                    matched_detection = DetectedCluster((md_1.pos_x+md_2.pos_x)/2., (md_1.pos_y+md_2.pos_y)/2., md_1.confidence, md_1.in_free_space)
                elif duplicates[propogated_track] in matched_tracks:
                    # Only one matched leg for this person 
                    md_1 = matched_tracks[duplicates[propogated_track]]
                    md_2 = propogated_track
                    matched_detection = DetectedCluster((md_1.pos_x+md_2.pos_x)/2., (md_1.pos_y+md_2.pos_y)/2., md_1.confidence, md_1.in_free_space)
                else:
                    # No legs matched for this person 
                    matched_detection = None
            else:
                if propogated_track in matched_tracks:
                    # Found a match for this non-person track
                    matched_detection = matched_tracks[propogated_track]
                else:
                    matched_detection = None

            if matched_detection:
                observations = np.array([matched_detection.pos_x, 
                                         matched_detection.pos_y])
                track.in_free_space = 0.8*track.in_free_space + 0.2*matched_detection.in_free_space
                track.confidence = 0.95*track.confidence + 0.05*matched_detection.confidence
                track.times_seen += 1
                track.last_seen = now
                track.seen_in_current_scan = True
            else: # propogated_track not matched to a detection
                # don't provide a measurement update for Kalman filter 
                # so send it a masked_array for its observations
                observations = np.ma.masked_array(np.array([0, 0]), mask=[1,1]) 
                track.seen_in_current_scan = False

            # Input observations to Kalman filter
            track.update(observations)

            # Check track for deletion
            if  track.is_person and track.confidence < self.confidence_threshold_to_maintain_track:
                tracks_to_delete.add(track)
                # rospy.loginfo("deleting due to low confidence")
            else:
                # Check track for deletion because covariance is too large
                cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                if cov > self.max_cov:
                    tracks_to_delete.add(track)
                    # rospy.loginfo("deleting because unseen for %.2f", (now - track.last_seen).to_sec())

        # Delete tracks that have been set for deletion
        for track in tracks_to_delete:
            track.deleted = True # Because the tracks are also pointed to in self.potential_leg_pairs, we have to mark them deleted so they can deleted from that set too
            self.objects_tracked.remove(track)

        # If detections were not matched, create a new track  
        for detect in detected_clusters:
            if not detect in matched_tracks.values():
                self.objects_tracked.append(ObjectTracked(detect.pos_x, detect.pos_y, now, detect.confidence, is_person=False, in_free_space=detect.in_free_space))

        # Do some leg pairing to create potential people tracks/leg pairs
        for track_1 in self.objects_tracked:
            for track_2 in self.objects_tracked:
                if (track_1 != track_2 
                    and track_1.id_num > track_2.id_num 
                    and (not track_1.is_person or not track_2.is_person) 
                    and (track_1, track_2) not in self.potential_leg_pairs
                ):
                    self.potential_leg_pairs.add((track_1, track_2))
                    self.potential_leg_pair_initial_dist_travelled[(track_1, track_2)] = (track_1.dist_travelled, track_2.dist_travelled)

        # We want to iterate over the potential leg pairs but iterating over the set <self.potential_leg_pairs> will produce arbitrary iteration orders.
        # This is bad if we want repeatable tests (but otherwise, it shouldn't affect performance).
        # So we'll create a sorted list and iterate over that.
        potential_leg_pairs_list = list(self.potential_leg_pairs)
        potential_leg_pairs_list.sort(key=lambda tup: (tup[0].id_num, tup[1].id_num))

        # Check if current leg pairs are still valid and if they should spawn a person
        leg_pairs_to_delete = set()
        for track_1, track_2 in potential_leg_pairs_list:
            # Check if we should delete this pair because 
            # - the legs are too far apart 
            # - or one of the legs has already been paired 
            # - or a leg has been deleted because it hasn't been seen for a while
            dist = ((track_1.pos_x - track_2.pos_x)**2 + (track_1.pos_y - track_2.pos_y)**2)**(1./2.)
            if (dist > self.max_leg_pairing_dist 
                or track_1.deleted or track_2.deleted
                or (track_1.is_person and track_2.is_person) 
                or track_1.confidence < self.confidence_threshold_to_maintain_track 
                or track_2.confidence < self.confidence_threshold_to_maintain_track
                ):
                    leg_pairs_to_delete.add((track_1, track_2))
                    continue
            
            # Check if we should create a tracked person from this pair
            # Three conditions must be met:
            # - both tracks have been matched to a cluster in the current scan
            # - both tracks have travelled at least a distance of <self.dist_travelled_together_to_initiate_leg_pair> since they were paired
            # - both tracks are in free-space
            if track_1.seen_in_current_scan and track_2.seen_in_current_scan:
                track_1_initial_dist, track_2_initial_dist = self.potential_leg_pair_initial_dist_travelled[(track_1, track_2)]
                dist_travelled = min(track_1.dist_travelled - track_1_initial_dist, track_2.dist_travelled - track_2_initial_dist)
                if (dist_travelled > self.dist_travelled_together_to_initiate_leg_pair 
                    and (track_1.in_free_space < self.in_free_space_threshold or track_2.in_free_space < self.in_free_space_threshold)
                    ):
                        if not track_1.is_person  and not track_2.is_person:
                            # Create a new person from this leg pair
                            self.objects_tracked.append(
                                ObjectTracked(
                                    (track_1.pos_x+track_2.pos_x)/2.,
                                    (track_1.pos_y+track_2.pos_y)/2., now,
                                    (track_1.confidence+track_2.confidence)/2.,
                                    is_person=True,
                                    in_free_space=0.)
                                )
                            track_1.deleted = True
                            track_2.deleted = True
                            self.objects_tracked.remove(track_1)
                            self.objects_tracked.remove(track_2)
                        elif track_1.is_person:
                            # Matched a tracked person to a tracked leg. Just delete the leg and the person will hopefully be matched next iteration
                            track_2.deleted = True
                            self.objects_tracked.remove(track_2)
                        else: # track_2.is_person:
                            # Matched a tracked person to a tracked leg. Just delete the leg and the person will hopefully be matched next iteration
                            track_1.deleted = True
                            self.objects_tracked.remove(track_1)
                        leg_pairs_to_delete.add((track_1, track_2))

        # Delete leg pairs set for deletion
        for leg_pair in leg_pairs_to_delete:
            self.potential_leg_pairs.remove(leg_pair)

        # Publish to rviz and /people_tracked topic.
        self.publish_tracked_objects(now)
        self.publish_tracked_people(now)


    """
    Publish markers of tracked objects to Rviz
    """
    def publish_tracked_objects(self, now):
        # Make sure we can get the required transform first:

        if self.use_scan_header_stamp_for_tfs:
            tf_time = now
            try:
                transform1 = self.buffer.lookup_transform(self.publish_people_frame, self.fixed_frame, tf_time, rclpy.duration(1.0))
                transform_available = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                transform_available = False
        else :
            tf_time = self.get_clock().now().to_msg()
            transform_available = self.buffer.can_transform(self.publish_people_frame, self.fixed_frame, tf_time)

        marker_id = 0
        if not transform_available:
            self.get_logger().info("Person tracker: tf not avaiable. Not publishing people")
        else:
            for track in self.objects_tracked:
                if track.is_person:
                    continue

                if self.publish_occluded or track.seen_in_current_scan:  # Only publish people who have been seen in current scan, unless we want to publish occluded people
                    # Get the track position in the <self.publish_people_frame> frame
                    ps = PointStamped()
                    ps.header.frame_id = self.fixed_frame
                    ps.header.stamp = tf_time
                    ps.point.x = track.pos_x
                    ps.point.y = track.pos_y
                    try:
                        ps = self.buffer.transform(ps, self.publish_people_frame)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue

                    # publish rviz markers
                    marker = Marker()
                    marker.header.frame_id =  self.publish_people_frame
                    marker.header.stamp = now
                    marker.ns = "objects_tracked"
                    if track.in_free_space < self.in_free_space_threshold:
                        marker.color.r = track.colour[0]
                        marker.color.g = track.colour[1]
                        marker.color.b = track.colour[2]
                    else:
                        marker.color.r = 0.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0
                    marker.color.a = 1.0
                    marker.pose.position.x = ps.point.x
                    marker.pose.position.y = ps.point.y
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = marker.CYLINDER
                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.20
                    marker.pose.position.z = 0.15

                    self.marker_pub.publish(marker)


                    # # Publish a marker showing distance travelled:
                    # if track.dist_travelled > 1:
                    #     marker.color.r = 1.0
                    #     marker.color.g = 1.0
                    #     marker.color.b = 1.0
                    #     marker.color.a = 1.0
                    #     marker.id = marker_id
                    #     marker_id += 1
                    #     marker.type = Marker.TEXT_VIEW_FACING
                    #     marker.text = str(round(track.dist_travelled,1))
                    #     marker.scale.z = 0.1            
                    #     marker.pose.position.z = 0.6
                    #     self.marker_pub.publish(marker)      

                    # Publish <self.confidence_percentile>% confidence bounds of track as an ellipse:
                    # cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                    # std = cov**(1./2.)
                    # gate_dist_euclid = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, std)                    
                    # marker.type = Marker.SPHERE
                    # marker.scale.x = 2*gate_dist_euclid
                    # marker.scale.y = 2*gate_dist_euclid
                    # marker.scale.z = 0.01   
                    # marker.color.r = 1.0
                    # marker.color.g = 1.0
                    # marker.color.b = 1.0                
                    # marker.color.a = 0.1
                    # marker.pose.position.z = 0.0
                    # marker.id = marker_id 
                    # marker_id += 1                    
                    # self.marker_pub.publish(marker)

            # Clear previously published track markers
            for m_id in range(marker_id, self.prev_track_marker_id):
                marker = Marker()
                marker.header.stamp = now
                marker.header.frame_id = self.publish_people_frame
                marker.ns = "oject_tracked"
                marker.id = m_id
                marker.action = marker.DELETE
                self.marker_pub.publish(marker)
            self.prev_track_marker_id = marker_id

    """
    Publish markers of tracked people to Rviz and to <people_tracked> topic
    """ 
    def publish_tracked_people(self, now):
        people_tracked_msg = PersonArray()
        people_tracked_msg.header.stamp = now
        people_tracked_msg.header.frame_id = self.publish_people_frame
        marker_id = 0

        # Make sure we can get the required transform first:
        if self.use_scan_header_stamp_for_tfs:
            tf_time = now
            try:
                transform = self.buffer.lookup_transform(self.fixed_frame, self.publish_people_frame, tf_time, rclpy.time.Duration(1.0))
                transform_available = True
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                transform_available = False
        else:
            tf_time = self.get_clock().now()
            transform_available = self.buffer.can_transform(self.fixed_frame, self.publish_people_frame, tf_time)

        marker_id = 0
        if not transform_available:
            self.get_logger().info("Person tracker: tf not avaiable. Not publishing people")
        else :
            for person in self.objects_tracked:
                if person.is_person == True:
                    if self.publish_occluded or person.seen_in_current_scan: # Only publish people who have been seen in current scan, unless we want to publish occluded people
                        # Get position in the <self.publish_people_frame> frame 
                        ps = PointStamped()
                        ps.header.frame_id = self.fixed_frame
                        ps.header.stamp = tf_time.to_msg()
                        ps.point.x = person.pos_x
                        ps.point.y = person.pos_y
                        try:
                            ps = self.buffer.transform(ps, self.publish_people_frame)
                        except:
                            self.get_logger().error("Not publishing people due to no transform from fixed_frame-->publish_people_frame")
                            continue

                        # pulish to people_tracked topic
                        new_person = Person()
                        new_person.pose.position.x = ps.point.x
                        new_person.pose.position.y = ps.point.y 
                        yaw = math.atan2(person.vel_y, person.vel_x)
                        quaternion = self.ToQuaternion(yaw, 0, 0)
                        new_person.pose.orientation.x = quaternion.x
                        new_person.pose.orientation.y = quaternion.y
                        new_person.pose.orientation.z = quaternion.z
                        new_person.pose.orientation.w = quaternion.w
                        new_person.id = person.id_num
                        people_tracked_msg.people.append(new_person)

                        # publish rviz markers       
                        # Cylinder for body
                        marker = Marker()
                        marker.header.frame_id = self.publish_people_frame
                        marker.header.stamp = now
                        marker.ns = "People_tracked"
                        marker.color.r = person.colour[0]
                        marker.color.g = person.colour[1]
                        marker.color.b = person.colour[2]
                        #marker.color.a = (rclpy.time.Duration(3) - (self.get_clock().now().to_msg() - person.last_seen)).nanoseconds()/rclpy.time.Duration(3).nanoseconds() + 0.1
                        marker.color.a = 1.0
                        marker.pose.position.x = ps.point.x 
                        marker.pose.position.y = ps.point.y
                        marker.id = marker_id
                        marker_id += 1
                        marker.type = Marker.CYLINDER
                        marker.scale.x = 0.2
                        marker.scale.y = 0.2
                        marker.scale.z = 1.2
                        marker.pose.position.z = 0.8
                        self.marker_pub.publish(marker)
                        
                        # Sphere for head shape
                        marker.type = Marker.SPHERE
                        marker.scale.x = 0.2
                        marker.scale.y = 0.2
                        marker.scale.z = 0.2
                        marker.pose.position.z = 1.5
                        marker.id = marker_id
                        marker_id += 1
                        self.marker_pub.publish(marker)
                        
                        # Text showing person's ID number
                        marker.color.r = 1.0
                        marker.color.g = 1.0
                        marker.color.b = 1.0
                        marker.color.a = 1.0
                        marker.id = marker_id
                        marker_id += 1
                        marker.type = Marker.TEXT_VIEW_FACING
                        marker.text = str(person.id_num)
                        marker.scale.z = 0.2
                        marker.pose.position.z = 1.7
                        self.marker_pub.publish(marker)
                        
                        # Arrow pointing in direction they're facing with magnitude proportional to speed
                        marker.color.r = person.colour[0]
                        marker.color.g = person.colour[1]
                        marker.color.b = person.colour[2]
                        #marker.color.a = ((rclpy.time.Duration(3.0) - (self.get_clock().now() - person.last_seen))/rclpy.time.Duration(3)) + 0.1
                        marker.color.a = 1.0
                        start_point = Point()
                        end_point = Point()
                        start_point.x = marker.pose.position.x 
                        start_point.y = marker.pose.position.y 
                        end_point.x = start_point.x + 0.5*person.vel_x
                        end_point.y = start_point.y + 0.5*person.vel_y
                        marker.pose.position.x = 0.
                        marker.pose.position.y = 0.
                        marker.pose.position.z = 0.1
                        marker.id = marker_id
                        marker_id += 1
                        marker.type = Marker.ARROW
                        marker.points.append(start_point)
                        marker.points.append(end_point)
                        marker.scale.x = 0.05
                        marker.scale.y = 0.1
                        marker.scale.z = 0.2
                        self.marker_pub.publish(marker)

                        # <self.confidence_percentile>% confidence bounds of person's position as an ellipse:
                        cov = person.filtered_state_covariances[0][0] + person.var_obs # cov_xx == cov_yy == cov
                        std = cov**(1./2.)
                        gate_dist_euclid = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, std)
                        marker.pose.position.x = ps.point.x
                        marker.pose.position.y = ps.point.y
                        marker.type = Marker.SPHERE
                        marker.scale.x = 2*gate_dist_euclid
                        marker.scale.y = 2*gate_dist_euclid
                        marker.scale.z = 0.01
                        marker.color.r = person.colour[0]
                        marker.color.g = person.colour[1]
                        marker.color.b = person.colour[2]
                        marker.color.a = 0.1
                        marker.pose.position.z = 0.0
                        marker.id = marker_id
                        marker_id += 1
                        self.marker_pub.publish(marker)

        # Clear previously published people markers
        for m_id in range(marker_id, self.prev_person_marker_id):
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.publish_people_frame
            marker.ns = "People_tracked"
            marker.id = m_id
            marker.action = marker.DELETE
            self.marker_pub.publish(marker)
        self.prev_person_marker_id = marker_id

        # Publish people tracked message
        self.people_tracked_pub.publish(people_tracked_msg)                    

                        
    def ToQuaternion (self, yaw, pitch, roll):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = 0.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q



def main(args=None):
    rclpy.init(args=args)
    node = KalmanMultiTrackerNode()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
        



