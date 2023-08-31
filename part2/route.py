#!/usr/local/bin/python3
# route.py : Find routes through maps
#
# Code by: name IU ID
#
# Based on skeleton code by V. Mathur and D. Crandall, January 2021
#


# !/usr/bin/env python3
import sys
from math import radians, cos, sin, asin, sqrt, tanh
import heapq
def read_road_segments():
    road_segments = dict()
    max_seg_len = 0
    max_speed_limit = 0 
    with open ('road-segments.txt','r') as f:
        for line in f.readlines():
            segment = line.strip().split(' ')
            if(int(segment[2]) > max_seg_len):
                max_seg_len = int(segment[2])
            if(int(segment[3]) > max_speed_limit):
                max_speed_limit = int(segment[3])
            city_1 = segment[0]
            city_2 = segment[1]
            #adding the road segments into a nested dictionary and adding the segment both ways  
            if not city_1 in road_segments:
                road_segments[city_1] = dict()
            if not city_2 in road_segments:
                road_segments[city_2] = dict()
            road_segments[city_1][city_2] = (float(segment[2]),float(segment[3]),segment[4])
            road_segments[city_2][city_1] = (float(segment[2]),float(segment[3]),segment[4])
            #print(road_segments)
    return (road_segments,max_speed_limit,max_seg_len)

#reading the gps.txt file and storing the cities as a dictionary
def load_city_details():
    cities = dict()
    with open ('city-gps.txt','r') as f:
        for line in f.readlines():
            city_data = line.strip().split(' ')
            city = city_data[0]
            lat = float(city_data[1])
            long = float(city_data[2])
            cities[city] = (lat, long)
    #print(cities['Indianapolis,_Indiana'])
    return cities

#using the haversine distance to get the min distance in miles between two coordinates 
#Reference : https://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
#The below function was copied from the stack overflow page. 
def get_min_distance_haversine(cord1, cord2):
    # converting decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [cord1[1], cord1[0], cord2[1], cord2[0]])
    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 3956 # Radius of earth in miles.
    distance  = c * r
    return distance

#Gets the heuristic for a given cost funtion. 
def get_priority(end_city, cost, present_state, cities, segments, max_speed_limit, max_seg_len, previous_priority):
    route_current, segment_current, distance_current, time_current = present_state
    current_city = route_current[-1]
    flag = 0
    if (len(route_current) >1):
        previous_city = route_current[-2]
    else:
        previous_city = ''
        flag = 1

    if current_city not in cities.keys() or end_city not in cities.keys():
        return previous_priority
    #gets the heursitic for a given city given distance as cost function
    if cost == "distance":
        cord1 = cities[current_city]
        cord2 = cities[end_city]
        distance_to_go = get_min_distance_haversine(cord1,cord2)
        return distance_current + distance_to_go
    #gets the heursitic for a given city given time as cost function   
    if cost == "time":
        cord1 = cities[current_city]
        cord2 = cities[end_city]
        distance_to_go = get_min_distance_haversine(cord1,cord2)
        return time_current + (distance_to_go/max_speed_limit)
    #gets the heursitic for a given city given segments as cost function 
    if cost == "segments":
        if flag==1:
            heuristic = 0 
        else:
            seg_length = int(segments[previous_city][current_city][0])
            heuristic = float(seg_length/max_seg_len)
        return segment_current + heuristic
    #gets the heursitic for a given city given delivery as cost function 
    if cost == "delivery":
        if flag==1:
            estimated_time = 0 
        else:
            seg_speed = segments[previous_city][current_city][1]
            seg_length = segments[previous_city][current_city][0]
            previous_time = seg_length/seg_speed
            time_trip = time_current - previous_time #getting the time for the trip minus the segment with speed limit above 50 mph
            time_seg = seg_length/max_speed_limit
            if(seg_speed >= 50):
                estimated_time = time_trip + (time_seg + (tanh(seg_length/1000)*2*(time_seg + time_trip)))
            else:
                estimated_time = time_trip + time_seg
        return estimated_time

#generating the sucessors for a given city
def successors(road_segments,city):
    return road_segments[city].keys()

#Given a particluar route taken it calculates the delivery time.
def calc_delivery_time(route_taken, segments):
    first = 0
    second = 1 
    city1 = route_taken[first]
    city2 = route_taken[second]
    trip_time = 0
    while(second < len(route_taken)):
        city1 = route_taken[first]
        city2 = route_taken[second]
        speed_limit = segments[city1][city2][1]
        lenght = segments[city1][city2][0]
        time = float(lenght/speed_limit)
        if(speed_limit >= 50):
            del_time = time + (tanh(lenght/1000)*2*(time+trip_time))
            trip_time = del_time + trip_time
        else:
            trip_time = trip_time + time
        first += 1
        second += 1
    return trip_time

#Some post processing the route_found to get the output into the desired format required by main function. 
def route_post_processing(route_taken,segments):
    first = 0
    second = 1 
    route_formatted = [] 
    city1 = route_taken[first]
    city2 = route_taken[second]
    while(second < len(route_taken)):
        city1 = route_taken[first]
        city2 = route_taken[second]
        lenght = segments[city1][city2][0]
        name = segments [city1][city2][2]
        str1 = str(name) + " for " + str(lenght) +" miles"
        temp_tuple = (city2, str1)
        route_formatted.append(temp_tuple)
        first += 1
        second += 1
    return route_formatted

#Solves A* for the given start and end state 
#https://www.geeksforgeeks.org/heap-and-priority-queue-using-heapq-module-in-python/
#The heapq push and pop functions and how to use them were read from the above link
def get_route(start, end, cost):
    segments, max_speed_limit, max_seg_len = read_road_segments()
    cities = load_city_details()
    route_current = list ()
    fringe = list ()
    next_leg = ''
    route_current.append(start)
    segments_current = 0 
    distance_current = 0 
    time_current = 0
    previous_priority = 0
    #defining the intial fringe
    initial_fringe = (route_current, segments_current, distance_current, time_current)
    #Getting a priority for the intial fringe
    priority_index = get_priority(end, cost, initial_fringe, cities, segments, max_speed_limit, max_seg_len, previous_priority)
    #Creating dictionaries to keep track of the visited cities and the priority at which they were visited.
    priority_when_visited = dict()
    visited = dict ()
    heapq.heappush(fringe,(priority_index,initial_fringe))
    while(fringe):
        priority_index, (route_current, segments_current, distance_current, time_current) = heapq.heappop(fringe)
        previous_priority = priority_index
        source_city = route_current[-1]
        if source_city == end:
            delivery_time = calc_delivery_time(route_current ,segments)
            route_taken = route_post_processing(route_current,segments)
            final_dict = {"total-segments" : segments_current, 
            "total-miles" : distance_current, 
            "total-hours" : time_current, 
            "total-delivery-hours" : delivery_time, 
            "route-taken" : route_taken}
            return final_dict
        visited[source_city] = 1
        priority_when_visited[source_city] = priority_index
        next_leg = successors(segments,source_city)
        for next_stop in next_leg:
            seg_miles = segments[source_city][next_stop][0]
            seg_speed_limit = segments[source_city][next_stop][1]
            seg_time = seg_miles/seg_speed_limit
            successor_fringe_elem = (route_current + [next_stop], segments_current + 1, distance_current + seg_miles, time_current + seg_time)
            priority_index = get_priority(end, cost, successor_fringe_elem, cities, segments, max_speed_limit, max_seg_len, previous_priority)
            visited_check = visited.get(next_stop,0)
            if visited_check  and priority_index < priority_when_visited[next_stop]:
                    visited[next_stop] = 0
                    heapq.heappush(fringe, (priority_index, successor_fringe_elem))
            if not visited_check:
                    heapq.heappush(fringe, (priority_index, successor_fringe_elem))
    return None


# Please don't modify anything below this line
#
if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise(Exception("Error: expected 3 arguments"))

    (_, start_city, end_city, cost_function) = sys.argv
    if cost_function not in ("segments", "distance", "time", "delivery"):
        raise(Exception("Error: invalid cost function"))

    result = get_route(start_city, end_city, cost_function)
    #print(result)
    # Pretty print the route

    print("Start in %s" % start_city)
    for step in result["route-taken"]:
        print("   Then go to %s via %s" % step)

    print("\n          Total segments: %4d" % result["total-segments"])
    print("             Total miles: %8.3f" % result["total-miles"])
    print("             Total hours: %8.3f" % result["total-hours"])
    print("Total hours for delivery: %8.3f" % result["total-delivery-hours"])




