#include "vehicle.h"

Vehicle::Vehicle(ros::NodeHandle nh) : nh_(nh)
{
    //Publishers
    lateral_thrust_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/lateral_thrust_cmd", 1);
    lateral_thrust_angle_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/lateral_thrust_angle", 1);

    left_thrust_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1);
    left_thrust_angle_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_angle", 1);

    right_thrust_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1);
    right_thrust_angle_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_angle", 1);

    acknowledgement_ = nh_.advertise<std_msgs::Int64>("/acknowledgement", 1);

    //Subscribers
    data_packet_sub_ = nh_.subscribe("chatter", 1, dataPacketCallback);    //change topic name need to add, this------------------------
    vehicle_A_GPS_sub_ = nh_.subscribe("chatter", 1, vehicleAGPSCallback); //change topic name need to add, this------------------------
    vehicle_B_GPS_sub_ = nh_.subscribe("chatter", 1, vehicleBGPSCallback); //change topic name need to add, this------------------------

    l_thrust_.data = 0;
    r_thrust_.data = 0;
}

void Vehicle::vehicleAGPSCallback()
{
    //retreive message.data

    vehicle_A_GPS_ = {0, 1}; //[longitude, latitude] change to message.data[0] etc
}

void Vehicle::vehicleBGPSCallback()
{
    //retreive message.data

    vehicle_B_GPS_ = {0, 1}; //[longitude, latitude] change to message.data[0] etc
}

double Vehicle::rangeCalc()
{
    double range = std::sqrt(std::pow(vehicle_A_GPS_[0] - vehicle_B_GPS_[0], 2) + std::pow(vehicle_A_GPS_[1] - vehicle_B_GPS_[1], 2));
    if (data_packet_[0] >= 1)
    {
        speed_of_sound_ = data_packet_[2];
    }
    else
    {
        speed_of_sound_ = 0;
    }
    double time_delta = range / speed_of_sound_;
    std::this_thread::sleep_for(std::chrono::milliseconds(time_delta * 1000));
    return range;
}

// short Vehicle::oldRangeCalc()
// {
//     short data_packet_time = 0;
//     short speed_of_sound_ = 0;

//     if (data_packet_[0] >= 1)
//     {
//         data_packet_time = data_packet_[1];
//         speed_of_sound_ = data_packet_[2];
//     }

//     short timenow = std::chrono::system_clock::now().time_since_epoch().count();
//     short delta_time = std::abs(timenow - data_packet_time);
//     short range = speed_of_sound_ * delta_time;
//     return range;
// }

void Vehicle::mainFunction()
{
    std::cout << "initialised" << std::endl;
    control();
}

void Vehicle::dataPacketCallback()
{
    data_packet_.clear();
    //Will need to add rosmsg link message here/ save the message
    short packet_number, x, y, z, timestamp, speed_of_sound, depth, distance_moved, direction_moved;

    switch (packet_number)
    {
    case 0:
        data_packet_ = {0};
        break;
    case 1:
        // 1st Data Packet
        // Packet number
        // Timestamp
        // Speed of sound of vehicle A
        // Vehicle A Depth
        // Vehicle A to point of interest pose: (x,y,z)
        data_packet_ = {packet_number, timestamp, speed_of_sound, depth, x, y, z};
        break;
    case (packet_number >= 2):
        // Data Packet
        // Packet number
        // Timestamp
        // Speed of sound of vehicle A
        // Vehicle A Depth
        // Distance and direction moved since the last transmission
        data_packet_ = {packet_number, timestamp, speed_of_sound, depth, distance_moved, direction_moved};
        break;
    default:
        break;
    }
}

void Vehicle::acknowledgement()
{
    //Change from publisher to service

    //Send the current packet number as acknowledgement
    acknowledgement_data_.data = data_packet_[0];
    acknowledgement_.publish(acknowledgement_data_);
}

short Vehicle::explorationVehicleVector(void)
{
    //Vehicle A movement vector, name check

    //Get GPS at each range check

    //Pushback to vector

    //Check how many values the vector has

    //subtract each segment to get the vectors

    return exploration_movement_vector;
}

void Vehicle::localisation(void)
{
    //Might need to check/wait for next data packet before localising
    range_circles.pushback(rangeCalc());
    movement_vectors.pushback(explorationVehicleVector());

    // Ensure only 3 range circles are used
    if (range_circles.size() > 3)
    {
        range_circles.erase(range_circles.begin());
    }

    // Movement vectors in between each range circles
    if (movement_vectors.size() > 2)
    {
        movement_vectors.erase(range_circles.begin());
    }

    // [A11, A12] = vectorInCircles(movement vector of A, distance from investigate vehicle to range circle 1 , B1_true, D2, B2_true)

    investigation_vector = 0;

    // For loop

    explore_vector = movement_vector.at(i + 1);
    net_vector = explore_vector - investigation_vector;



    v = v - (b2 - b1);
    theta = acos((d1 ^ 2 + norm(v) ^ 2 - d2 ^ 2) / (2 * d1 * norm(v)));
    vector_angle = atan2(v(2), v(1));
    [ x1, y1 ] = pol2cart(theta + vector_angle, d1);
    [ x2, y2 ] = pol2cart(-theta + vector_angle, d1);
    solution1 = [ -x1, -y1 ] + b1; % first solution for A1
    solution2 = [-x2, -y2] + b1; % second solution for A1
}

void Vehicle::control()
{
    lat_thrust_.data = 1;

    l_thrust_.data = 1;
    r_thrust_.data = 1;

    l_thrust_angle_.data = 0.1;
    r_thrust_angle_.data = 0.1;
    double increment;
    double increment2 = 0.1;

    // float test = r_thrust_.data - l_thrust_.data;

    ros::Rate loop_rate(10);

    while (ros::ok)
    {
        // test = test / (test * 1.1);

        std::cout << l_thrust_angle_.data << std::endl;

        // increment2 = (increment2)*inc/1.001;
        // increment = increment2 - 0.1;

        l_thrust_angle_.data = l_thrust_angle_.data * (1 - (0.001 * l_thrust_angle_.data / 0.1)); // calculates current thrut angle as percentage of starting thrust angle, then calculates 0.001 (0.1%) of it. Therefore next thrust angle is 1- 0.001 = 99.9 percent of the previous.
        r_thrust_angle_.data = r_thrust_angle_.data * (1 - (0.001 * r_thrust_angle_.data / 0.1));

        // l_thrust_angle_.data = l_thrust_angle_.data - 0.0001;
        // r_thrust_angle_.data = r_thrust_angle_.data - 0.0001;

        // // l_thrust_angle_.data = l_thrust_angle_.data - 0.001;
        // // r_thrust_angle_.data = r_thrust_angle_.data - 0.001;

        lateral_thrust_.publish(lat_thrust_);
        // lateral_thrust_angle_.publish(msg);

        left_thrust_.publish(l_thrust_);
        left_thrust_angle_.publish(l_thrust_angle_);

        right_thrust_.publish(r_thrust_);
        right_thrust_angle_.publish(r_thrust_angle_);

        // r_thrust_.data = 1 - test;

        loop_rate.sleep();
    }
}

Vehicle::~Vehicle()
{
}