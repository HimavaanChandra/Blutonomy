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

    lateral_thrust_2_ = nh_.advertise<std_msgs::Float32>("/wamv2/thrusters/lateral_thrust_cmd", 1);
    lateral_thrust_angle_2_ = nh_.advertise<std_msgs::Float32>("/wamv2/thrusters/lateral_thrust_angle", 1);

    left_thrust_2_ = nh_.advertise<std_msgs::Float32>("/wamv2/thrusters/left_thrust_cmd", 1);
    left_thrust_angle_2_ = nh_.advertise<std_msgs::Float32>("/wamv2/thrusters/left_thrust_angle", 1);

    right_thrust_2_ = nh_.advertise<std_msgs::Float32>("/wamv2/thrusters/right_thrust_cmd", 1);
    right_thrust_angle_2_ = nh_.advertise<std_msgs::Float32>("/wamv2/thrusters/right_thrust_angle", 1);

    acknowledgement_ = nh_.advertise<std_msgs::Int64>("/acknowledgement", 1);

    data_packet_pub_ = nh_.advertise<data_packet_msg_cpp::data_packet>("/data_packet", 1);


    //Subscribers
    vehicle_A_GPS_sub_ = nh_.subscribe("/wamv/sensors/gps/gps/fix", 1, &Vehicle::vehicleAGPSCallback, this);
    vehicle_B_GPS_sub_ = nh_.subscribe("/wamv2/sensors/gps/gps/fix", 1, &Vehicle::vehicleBGPSCallback, this);
    
    data_packet_sub_ = nh_.subscribe("/data_packet", 1, &Vehicle::dataPacketCallback, this);

    l_thrust_.data = 0;
    r_thrust_.data = 0;
    packet_number_ = 0;
    localised_ = false;
}

void Vehicle::vehicleAGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    vehicle_A_GPS_ = {msg->latitude, msg->longitude};
}

void Vehicle::vehicleBGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    vehicle_B_GPS_ = {msg->latitude, msg->longitude};
}

// double Vehicle::rangeCalc(void)
// {
//     double range = std::sqrt(std::pow(vehicle_A_GPS_[0] - vehicle_B_GPS_[0], 2) + std::pow(vehicle_A_GPS_[1] - vehicle_B_GPS_[1], 2));
//     if (data_packet_[0] >= 1)
//     {
//         speed_of_sound_ = data_packet_[2];
//     }
//     else
//     {
//         speed_of_sound_ = 0;
//     }
//     double time_delta = range / speed_of_sound_;
//     int int_time = time_delta * 1000;
//     std::this_thread::sleep_for(std::chrono::milliseconds(int_time));
//     std::cout << "Range: " << range << "Time: " << time_delta << std::endl;
//     return range;
// }

void Vehicle::mainFunction(void)
{
    std::cout << "Program Start" << std::endl;

    // Spiral pattern is running in separate thread

    // Wait a certain (possibly random) amount of time before finding point of interest
    int point_of_interest_delay = 5;
    std::this_thread::sleep_for(std::chrono::seconds(point_of_interest_delay));
    int transmission_delay = 5; // change back to 60 ------------------------------------------------

    while (!localised_)
    {
        std::cout << "localised while loop" << std::endl;
        publishDataPacket();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (data_packet_.size() > 1)
        {
            std::cout << "packet number: " << data_packet_.at(0) << std::endl;
            if (data_packet_.at(0) > 1)
            {
                localisation();
            }
        }
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(transmission_delay));
    }

    std::vector<double> goal = {resultant_.at(0) + vehicle_B_GPS_.at(0), resultant_.at(1) + vehicle_B_GPS_.at(1)};

    std::vector<double> distance_to_goal = {(vehicle_A_GPS_.at(0) - vehicle_B_GPS_.at(0)) * lat_to_meters, (vehicle_A_GPS_.at(1) - vehicle_B_GPS_.at(1)) * long_to_meters};
    double distance_to_goal_mag = 1;
    std::cout << "localised" << std::endl;
    std::cout << "localised" << std::endl;

    std::cout << "gps_x" << distance_to_goal.at(0) << std::endl;
    std::cout << "gps_y"<< distance_to_goal.at(1) << std::endl;
    std::cout << "resultant_x"<< resultant_.at(0) << std::endl;
    std::cout << "resultant_y"<< resultant_.at(1) << std::endl;

    std::cout << "localised" << std::endl;

    while (distance_to_goal_mag > 0.0001)
    {
        // distance_to_goal = {resultant_.at(0), resultant_.at(1)};
        distance_to_goal = {(vehicle_A_GPS_.at(0) - vehicle_B_GPS_.at(0)) * lat_to_meters, (vehicle_A_GPS_.at(1) - vehicle_B_GPS_.at(1)) * long_to_meters};
        distance_to_goal_mag = sqrt(pow(distance_to_goal.at(0), 2) + pow(distance_to_goal.at(1), 2));
        // distance_to_goal_mag = sqrt(pow(resultant_.at(0), 2) + pow(resultant_.at(1), 2));
        // purePursuit(resultant_.at(0), distance_to_goal_mag);
        purePursuit(distance_to_goal.at(0), distance_to_goal_mag);

        // std::cout << distance_to_goal_mag << std::endl;
        // publishDataPacket();
        // if (data_packet_.size() > 1)
        // {
        //     std::cout << "packet number: " << data_packet_.at(0) << std::endl;
        //     if (data_packet_.at(0) > 1)
        //     {
        //         localisation();
        //     }
        // }
        // std::cout << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(transmission_delay));
    }

    std::cout << "Program Completed" << std::endl;
}

double Vehicle::simulateRange(void)
{
    double time_now = std::chrono::system_clock::now().time_since_epoch().count();
    //Range is wrong, need to convert to metres from longitude and lattitude
    double range = std::sqrt(std::pow((vehicle_A_GPS_[0] - vehicle_B_GPS_[0]) * lat_to_meters, 2) + std::pow((vehicle_A_GPS_[1] - vehicle_B_GPS_[1]) * long_to_meters, 2));
    double time_delta = range / speed_of_sound_;
    int int_time = time_delta * 1000;
    std::this_thread::sleep_for(std::chrono::milliseconds(int_time));
    std::cout << "Range sent: " << range << "Time: " << time_delta << std::endl;
    // return time_now;
    return range;
}

double Vehicle::rangeCalc(double time_sent)
{
    double time_now = std::chrono::system_clock::now().time_since_epoch().count();
    double delta_time = time_now - time_sent;
    short range = speed_of_sound_ * delta_time;
    std::cout << "Range received: " << range << "Time delta: " << delta_time << std::endl;
    return range;
}

void Vehicle::publishDataPacket()
{
    //Vehicle A to point of interest coords
    float POI_lattitude = 0;
    float POI_longitude = 0;
    float z = 0;

    //Vehicle A movement vector
    vehicle_A_GPS_history_.push_back(vehicle_A_GPS_);
    vehicle_B_GPS_history_.push_back(vehicle_B_GPS_);
    std::cout << "gps history size: " << vehicle_A_GPS_history_.size() / 2 << std::endl;
    std::vector<float> A_moved = explorationVehicleVector();
    if (A_moved.size() >= 2)
    {
        float A_latitude_moved = A_moved.at(0);
        float A_longitude_moved = A_moved.at(1);

        //Run range function here then send delay and speed of sound?
        // float timestamp = simulateRange();
        float range = simulateRange();
        float depth = 0; //since on surface
        Blutonomy::data_packet msg; 
        packet_number_++;
        if (packet_number_ == 0)
        {
            // Send range instead of timestamp for sim
            // data_packet_ = {packet_number_, timestamp, speed_of_sound_, depth, POI_lattitude, POI_longitude, z};    
            msg.packet_number.data = packet_number_;
            msg.range.data = range;
            msg.speed_of_sound.data = speed_of_sound_;
            msg.depth.data = 0;
            msg.POI_lattitude.data = 0;
            msg.POI_longitude.data = 0;
            msg.z.data = 0;
        }
        else if (packet_number_ >= 1)
        {
            // Send range instead of timestamp for sim
            // data_packet_ = {packet_number_, timestamp, speed_of_sound_, depth, A_latitude_moved, A_longitude_moved};  
            msg.packet_number.data = packet_number_;
            msg.range.data = range;
            msg.speed_of_sound.data = speed_of_sound_;
            msg.depth.data = 0;
            msg.A_latitude_moved.data = 0;
            msg.A_longitude_moved.data = 0;

        }
        std::cout << "packet sent number: " << data_packet_.at(0) << std::endl;

        //ros publish datapacket
        data_packet_pub_.publish(msg);
        // ros::spinOnce(); //might need this
    }
}

//May need 2 callback
void Vehicle::dataPacketCallback(const data_packet_msg_cpp::data_packet::ConstPtr& msg)
{
    data_packet_.clear();
    if (packet_number == 0)
    {
        data_packet_ = {0};
    }
    else if (packet_number == 1)
    {
        // 1st Data Packet
        data_packet_ = {msg->packet_number.data, msg->range.data, msg->speed_of_sound.data, msg->depth.data, msg->POI_lattitude.data, msg->POI_longitude.data, msg->z.data};
        vehicle_A_GPS_history_.push_back(vehicle_A_GPS_);
        vehicle_B_GPS_history_.push_back(vehicle_B_GPS_);
    }
    else if (packet_number >= 2)
    {
        // Other Data Packets
        data_packet_ = {msg->packet_number.data, msg->range.data, msg->speed_of_sound.data, msg->depth.data, msg->A_latitude_moved.data, msg->A_longitude_moved.data};
        vehicle_A_GPS_history_.push_back(vehicle_A_GPS_);
        vehicle_B_GPS_history_.push_back(vehicle_B_GPS_);
    }
}

void Vehicle::acknowledgement(void)
{
    //Change from publisher to service

    //Send the current packet number as acknowledgement
    acknowledgement_data_.data = data_packet_[0];
    acknowledgement_.publish(acknowledgement_data_);
}

std::vector<float> Vehicle::explorationVehicleVector(void)
{
    //Vehicle A movement vector, function name check
    //pGet last 2 gps points
    int vector_size = vehicle_A_GPS_history_.size();
    std::vector<float> exploration_movement_vector;
    exploration_movement_vector.clear();
    if (vector_size > 1)
    {
        //latitude
        float latitude = vehicle_A_GPS_history_.at(vector_size - 1).at(0) - vehicle_A_GPS_history_.at(vector_size - 2).at(0);
        exploration_movement_vector.push_back(latitude * lat_to_meters);
        //longitude
        float longitude = vehicle_A_GPS_history_.at(vector_size - 1).at(1) - vehicle_A_GPS_history_.at(vector_size - 2).at(1);
        exploration_movement_vector.push_back(longitude * long_to_meters);
        std::cout << "latitude, longitude: " << latitude << ", " << longitude << std::endl;
    }

    return exploration_movement_vector;
}

std::vector<std::vector<float>> Vehicle::vectorLocalisation(std::vector<float> net_vector, double d1, double d2)
{

    double net_vector_mag = sqrt(pow(net_vector.at(0), 2) + pow(net_vector.at(1), 2) /*, pow(net_vector.at(2), 2)*/);

    double theta = acos((pow(d1, 2) + pow(net_vector_mag, 2) - pow(d2, 2)) / (2 * d1 * net_vector_mag));
    double vector_angle = atan2(net_vector.at(1), net_vector.at(0));
    // double vector_angle = 30;

    double net_angle_1 = theta + vector_angle;
    double net_angle_2 = -theta + vector_angle;

    double x1 = d1 * cos(net_angle_1);
    double y1 = d1 * sin(net_angle_1);
    std::vector<float> solution_1 = {x1, y1};

    double x2 = d1 * cos(net_angle_2);
    double y2 = d1 * sin(net_angle_2);
    std::vector<float> solution_2 = {x2, y2};

    return {solution_1, solution_2};
}

void Vehicle::localisation(void)
{
    //Might need to check/wait for next data packet before localising

    int number_of_distance_circles = 3;

    solutions.clear();
    net_vector.clear();

    // Add the most recent distance circle
    // range_circles.push_back(rangeCalc(data_packet_.at(1)));
    range_circles.push_back(data_packet_.at(1));
    std::vector<float> A_moved = {data_packet_.at(4), data_packet_.at(5)};
    // Add the most recent movement vector

    movement_vectors.push_back(A_moved);
    std::cout << "localisation function start" << std::endl;
    if (movement_vectors.size() > number_of_distance_circles - 1)
    {
        movement_vectors.erase(movement_vectors.begin());
    }
    std::cout << "yeet: " << range_circles.size() << std::endl;

    // Localise when 3 range circles and remove the oldest range circle if more than 3 availble
    if (range_circles.size() > number_of_distance_circles)
    {
        range_circles.erase(range_circles.begin());

        std::cout << "yeetspegeet:" << std::endl;

        for (int i = 0; i < number_of_distance_circles - 1; i++)
        {

            // explore_vector = movement_vector.at(i);
            // net_vector = explore_vector - investigation_vector;

            //  Net vector combin vehicle A and B movement vectors
            net_vector = movement_vectors.at(i);
            // net_vector_mag.push_back(sqrt(pow(net_vector.at(0), 2) + pow(net_vector.at(1), 2)/*, pow(net_vector.at(2), 2)*/));

            // Radius from vehilce B to Vehicle A circles 1 and 2
            double d1 = range_circles.at(i);
            double d2 = range_circles.at(i + 1);

            // Push back solution 1 and 2 for each movement vector localisation. solutions .at(circle vector 1 or 2) .at(solution 1 or 2) . at(x or y)
            solutions.push_back(vectorLocalisation(net_vector, d1, d2));
        }
        std::cout << "yeetusspeegetus" << std::endl;

        double lowest_difference = 0;
        std::vector<int> lowest_index = {0, 0};

        for (int i = 0; i < number_of_distance_circles - 1; i++)
        {
            for (int j = 0; j < 2; j++)
            {

                // diffs(i,j) = norm((A1(i,:) + A1_A2) - A2(j,:))
                // Solutions .at(circle vector 1 or 2) .at(solution 1 or 2) . at(x or y)
                double vector_x = solutions.at(0).at(i).at(j) - solutions.at(1).at(i).at(j) + net_vector.at(0);
                double vector_y = solutions.at(0).at(i).at(j) - solutions.at(1).at(i).at(j) + net_vector.at(1);

                difference = sqrt(pow(vector_x, 2) + pow(vector_y, 2));

                if (difference < lowest_difference || lowest_difference == 0)
                {
                    lowest_difference = difference;
                    lowest_index = {i, j};
                }
            }
        }

        std::vector<float> solution1 = solutions.at(0).at(lowest_index.at(0));
        std::vector<float> solution2 = solutions.at(1).at(lowest_index.at(1));

        solutions.clear();

        resultant_ = {solution1.at(0) + solution2.at(0), solution1.at(1) + solution2.at(1)};

        localised_ = true;
    }
    // v = v - (b2 - b1);
    // theta = acos((d1 ^ 2 + norm(v) ^ 2 - d2 ^ 2) / (2 * d1 * norm(v)));
    // vector_angle = atan2(v(2), v(1));
    // [ x1, y1 ] = pol2cart(theta + vector_angle, d1);
    // [ x2, y2 ] = pol2cart(-theta + vector_angle, d1);
    // solution1 = [ -x1, -y1 ] + b1; % first solution for A1
    // solution2 = [-x2, -y2] + b1; % second solution for A1
    std::cout << "Localisation function end, localised_: " << localised_ << std::endl;
}

// centrDistance = latitude    Range = magnitude
void Vehicle::purePursuit(double centreDistance, double range)
{
    ros::Rate loop_rate(10);
    // Calculating maximum angular velocity for velocity control
    float gamma = ((2 * centreDistance) / std::pow(range, 2)) * 10;
    float linear_velocity = 1;
    double angular_velocity = linear_velocity * gamma * 5;
    // if (gamma < 0)
    // {
    //     if (angular_velocity < 0)
    //     {
    //         gamma = -gamma;
    //         angular_velocity = -angular_velocity;
    //     }
    // }
    // else
    // {
    //     if (angular_velocity > 0)
    //     {
    //         gamma = -gamma;
    //         angular_velocity = -angular_velocity;
    //     }
    // }

    if (linear_velocity > 1)
    {
        linear_velocity = 1;
    }

    while (linear_velocity > 1 /*|| angular_velocity > 1*/)
    {
        // angular_velocity *= 0.99;
        linear_velocity *= 0.99;
    }
    l_thrust_2_.data = linear_velocity;
    r_thrust_2_.data = linear_velocity;

    l_thrust_angle_2_.data = gamma;
    r_thrust_angle_2_.data = gamma;

    left_thrust_2_.publish(l_thrust_2_);
    left_thrust_angle_2_.publish(l_thrust_angle_2_);

    right_thrust_2_.publish(r_thrust_2_);
    right_thrust_angle_2_.publish(r_thrust_angle_2_);

    loop_rate.sleep();
    // left_thrust_.publish(linear_velocity);
    // left_thrust_angle_.publish(gamma);

    // right_thrust_.publish(linear_velocity);
    // right_thrust_angle_.publish(gamma);

    // robot_.twist_.linear.x = linear_velocity;
    // robot_.twist_.angular.z = angular_velocity;
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

        // std::cout << l_thrust_angle_.data << std::endl;

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