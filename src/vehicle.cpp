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
    // data_packet_sub_ = nh_.subscribe("/chatter", 1, &Vehicle::dataPacketCallback, this);    //Change topic need to add, this------------------------
    vehicle_A_GPS_sub_ = nh_.subscribe("/wamv/sensors/gps/gps/fix", 1, &Vehicle::vehicleAGPSCallback, this);
    vehicle_B_GPS_sub_ = nh_.subscribe("/wamv2/sensors/gps/gps/fix", 1, &Vehicle::vehicleBGPSCallback, this);

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

double Vehicle::rangeCalc(void)
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
    int int_time = time_delta * 1000;
    std::this_thread::sleep_for(std::chrono::milliseconds(int_time));
    std::cout << "Range: " << range << "Time: " << time_delta << std::endl;
    return range;
}

void Vehicle::mainFunction(void)
{
    std::cout << "Program Start" << std::endl;

    // Spiral pattern is running in separate thread

    // Wait a certain (possibly random) amount of time before finding point of interest
    int point_of_interest_delay = 10;
    std::this_thread::sleep_for(std::chrono::seconds(point_of_interest_delay));
    int transmission_delay = 60;

    while (!localised_)
    {
        publishDataPacket();
        if (data_packet_.at(0) > 1)
        {
            localisation();
        }
        // Wait 1 minute
        std::this_thread::sleep_for(std::chrono::seconds(transmission_delay));
    }

    // Calculate position of vehicle A

    // Work out pose to point of interest to vehicle B

    // Drive vehicle B to point

    std::vector<double> goal = {resultant_.at(0) + vehicle_B_GPS_.at(0), resultant_.at(1) + vehicle_B_GPS_.at(1)};

    std::vector<double> distance_to_goal;
    double distance_to_goal_mag = 1;

    while (distance_to_goal_mag > 0.0001)
    {
        distance_to_goal = {goal.at(0) - vehicle_B_GPS_.at(0), goal.at(1) - vehicle_B_GPS_.at(1)};
        distance_to_goal_mag = sqrt(pow(distance_to_goal.at(0), 2) + pow(distance_to_goal.at(1), 2);
        purePursuit(distance_to_goal.at(0), distance_to_goal_mag);
    }

    std::cout << "Program Completed" << std::endl;
}

double Vehicle::simulateRange(void)
{
    double time_now = std::chrono::system_clock::now().time_since_epoch().count();
    double range = std::sqrt(std::pow(vehicle_A_GPS_[0] - vehicle_B_GPS_[0], 2) + std::pow(vehicle_A_GPS_[1] - vehicle_B_GPS_[1], 2));
    double time_delta = range / speed_of_sound_;
    int int_time = time_delta * 1000;
    std::this_thread::sleep_for(std::chrono::milliseconds(int_time));
    std::cout << "Range: " << range << "Time: " << time_delta << std::endl;
    return time_now;
}

double Vehicle::rangeCalc(double time_sent)
{
    double time_now = std::chrono::system_clock::now().time_since_epoch().count();
    double delta_time = time_now - time_sent;
    short range = speed_of_sound_ * delta_time;
    return range;
}

void Vehicle::publishDataPacket()
{
    //Vehicle A to point of interest coords
    float POI_lattitude = 0;
    float POI_longitude = 0;
    float z = 0;

    //Vehicle A movement vector
    std::vector<float> A_moved = explorationVehicleVector();
    float A_latitude_moved = A_moved.at(0);
    float A_longitude_moved = A_moved.at(1);

    //Run range function here then send delay and speed of sound?
    float timestamp = simulateRange();
    float depth = 0; //since on surface

    std::vector<float> data_packet = {0};

    if (packet_number_ == 0)
    {
        packet_number_++;
        data_packet = {packet_number_, timestamp, speed_of_sound_, depth, POI_lattitude, POI_longitude, z};
    }
    else if (packet_number_ >= 1)
    {
        packet_number_++;
        data_packet = {packet_number_, timestamp, speed_of_sound_, depth, A_latitude_moved, A_longitude_moved};
    }

    //ros publish datapacket
}

void Vehicle::dataPacketCallback()
{
    //Redo once message is constructed
    data_packet_.clear();
    //Will need to add rosmsg link message here/ save the message
    float packet_number, POI_lattitude, POI_longitude, z, timestamp, speed_of_sound, depth, A_latitude_moved, A_longitude_moved;

    if (packet_number == 0)
    {
        data_packet_ = {0};
    }
    else if (packet_number == 1)
    {
        // 1st Data Packet
        // Packet number
        // Timestamp
        // Speed of sound of vehicle A
        // Vehicle A Depth
        // Vehicle A to point of interest pose: (x,y,z)
        data_packet_ = {packet_number, timestamp, speed_of_sound, depth, POI_lattitude, POI_longitude, z};
        vehicle_A_GPS_history_.push_back(vehicle_A_GPS_);
        vehicle_B_GPS_history_.push_back(vehicle_B_GPS_);
    }
    else if (packet_number >= 2)
    {
        // Data Packet
        // Packet number
        // Timestamp
        // Speed of sound of vehicle A
        // Vehicle A Depth
        // Distance and direction moved since the last transmission
        data_packet_ = {packet_number, timestamp, speed_of_sound, depth, A_latitude_moved, A_longitude_moved};
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

std::vector<double> Vehicle::explorationVehicleVector(void)
{
    //Vehicle A movement vector, function name check
    //pGet last 2 gps points
    int vector_size = vehicle_A_GPS_history_.size();
    std::vector<double> exploration_movement_vector;
    if (vector_size > 1)
    {
        //latitude
        exploration_movement_vector.push_back(vehicle_A_GPS_history_.at(0).at(vector_size - 1) - vehicle_A_GPS_history_.at(0).at(vector_size - 2));
        //longitude
        exploration_movement_vector.push_back(vehicle_A_GPS_history_.at(vector_size - 1).at(0) - vehicle_A_GPS_history_.at(vector_size - 2).at(0));
    }

    //Pushback to vector

    //Check how many values the vector has

    //subtract each segment to get the vectors

    //Add random tolerance to ranges - google drive - 0.1cm per second,
    // unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine generator(seed);
    // create a uniform distribution between 0 and 10 and draw elements from it
    // std::uniform_real_distribution<double> distribution(max_radius, 0);
    // std::normal_distribution<double> distribution(4, 5);
    // double elements = distribution(generator); // Generates a random double

    //To implement, will need to see delta time, then multiply this buy a random value to 0.1 in seconds to both vector components

    return exploration_movement_vector;
}

std::vector<std::vector<double>> Vehicle::vectorLocalisation(std::vector<double> net_vector, double d1, double d2)
{

    double net_vector_mag = sqrt(pow(net_vector.at(0), 2) + pow(net_vector.at(1), 2) /*, pow(net_vector.at(2), 2)*/);

    double theta = acos((pow(d1, 2) + pow(net_vector_mag, 2) - pow(d2, 2)) / (2 * d1 * net_vector_mag));
    double vector_angle = atan2(net_vector.at(1), net_vector.at(0));
    // double vector_angle = 30;

    double net_angle_1 = theta + vector_angle;
    double net_angle_2 = -theta + vector_angle;

    double x1 = d1 * cos(net_angle_1);
    double y1 = d1 * sin(net_angle_1);
    std::vector<double> solution_1 = {x1, y1};

    double x2 = d1 * cos(net_angle_2);
    double y2 = d1 * sin(net_angle_2);
    std::vector<double> solution_2 = {x2, y2};

    return {solution_1, solution_2};
}

void Vehicle::localisation(void)
{
    //Might need to check/wait for next data packet before localising

    int number_of_distance_circles = 3;

    solutions.clear();
    net_vector.clear();

    // Add the most recent distance circle
    range_circles.push_back(rangeCalc(data_packet_.at(1)));

    std::vector<float> A_moved = {data_packet_.at(4), data_packet_.at(5)};
    // Add the most recent movement vector

    movement_vectors.push_back(A_moved);

    if (movement_vectors.size() > number_of_distance_circles - 1)
    {
        movement_vectors.erase(movement_vectors.begin());
    }

    // Localise when 3 range circles and remove the oldest range circle if more than 3 availble
    if (range_circles.size() > number_of_distance_circles)
    {
        range_circles.erase(range_circles.begin());
        solutions.erase(solutions.begin());

        for (int i = 0; i < number_of_distance_circles - 2; i++)
        {

            // explore_vector = movement_vector.at(i);
            // net_vector = explore_vector - investigation_vector;

            //  Net vector combin vehicle A and B movement vectors
            net_vector.push_back(movement_vectors.at(i));
            // net_vector_mag.push_back(sqrt(pow(net_vector.at(0), 2) + pow(net_vector.at(1), 2)/*, pow(net_vector.at(2), 2)*/));

            // Radius from vehilce B to Vehicle A circles 1 and 2
            double d1 = range_circles.at(i);
            double d2 = range_circles.at(i + 1);

            // Push back solution 1 and 2 for each movement vector localisation. solutions .at(circle vector 1 or 2) .at(solution 1 or 2) . at(x or y)
            solutions.push_back(vectorLocalisation(net_vector, d1, d2));
        }

        double lowest_difference = 0;
        std::vector<double> lowest_index = {0, 0};

        for (int i = 0; i < number_of_distance_circles - 2; i++)
        {
            for (int j = 0; j < 1; j++)
            {

                // diffs(i,j) = norm((A1(i,:) + A1_A2) - A2(j,:))
                // Solutions .at(circle vector 1 or 2) .at(solution 1 or 2) . at(x or y)
                std::vector<double> vector_x = solutions.at(0).at(i).at(0) - solutions.at(1).at(j).at(0) + net_vector.at(0);
                std::vector<double> vector_y = solutions.at(0).at(i).at(1) - solutions.at(1).at(j).at(1) + net_vector.at(1);

                difference = sqrt(pow(vector_x, 2) + pow(vector_y, 2));

                if (difference < lowest_difference || lowest_difference = 0)
                {
                    lowest_difference = difference;
                    lowest_index = {i, j};
                }
            }
        }

        std::vector<double> solution1 = solutions.at(0).at(lowest_index(i));
        std::vector<double> solution2 = solutions.at(1).at(lowest_index(j));

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
}

// centrDistance = latitude    Range = magnitude
void Vehicle::purePursuit(double centreDistance, double range)
{
    // Calculating maximum angular velocity for velocity control
    double gamma = (2 * std::sin(centreDistance)) / std::pow(range, 2);
    double linear_velocity = 0.22;
    //   double angular_velocity = linear_velocity * gamma * 5;
    //   if (gamma < 0)
    //   {
    //     if (angular_velocity < 0)
    //     {
    //       angular_velocity = -angular_velocity;
    //     }
    //   }
    //   else
    //   {
    //     if (angular_velocity > 0)
    //     {
    //       angular_velocity = -angular_velocity;
    //     }
    //   }

    if (linear_velocity > 1)
    {
        linear_velocity = 1;
    }

    while (linear_velocity > 1 || angular_velocity > 1)
    {
        // angular_velocity *= 0.99;
        linear_velocity *= 0.99;
    }

    left_thrust_.publish(linear_velocity);
    left_thrust_angle_.publish(gamma);

    right_thrust_.publish(linear_velocity);
    right_thrust_angle_.publish(gamma);

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