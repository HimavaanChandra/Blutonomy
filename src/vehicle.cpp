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
}

void Vehicle::vehicleAGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    vehicle_A_GPS_ = {msg->longitude, msg->latitude};
}

void Vehicle::vehicleBGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    vehicle_B_GPS_ = {msg->longitude, msg->latitude};
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
//     short range = speed_of_sound_ * delta_time;-
//     return range;
// }

void Vehicle::mainFunction(void)
{
    std::cout << "initialised" << std::endl;
    control();

    // Start spiral pattern

    // Wait a certain (possibly random) amount of time before finding point of interest

    // Send first range circle

    // Wait 1-2 mins

    // Send second range circle

    // Perform first localisation calc

    // wait 1-2 mins

    // Perform second localisation calc

    // Calculate position of vehicle A

    // Work out pose to point of interest to vehicle B

    //Drive vehicle B to point

}

void Vehicle::dataPacketCallback()
{
    data_packet_.clear();
    //Will need to add rosmsg link message here/ save the message
    float packet_number, x, y, z, timestamp, speed_of_sound, depth, distance_moved, direction_moved;

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
        data_packet_ = {packet_number, timestamp, speed_of_sound, depth, x, y, z};
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
        data_packet_ = {packet_number, timestamp, speed_of_sound, depth, distance_moved, direction_moved};
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
        //longitude
        exploration_movement_vector.push_back(vehicle_A_GPS_history_.at(vector_size - 1).at(0) - vehicle_A_GPS_history_.at(vector_size - 2).at(0));
        //latitude
        exploration_movement_vector.push_back(vehicle_A_GPS_history_.at(0).at(vector_size - 1) - vehicle_A_GPS_history_.at(0).at(vector_size - 2));
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
    range_circles.push_back(rangeCalc());
    movement_vectors.push_back(explorationVehicleVector());

    // Ensure only 3 range circles are used
    if (range_circles.size() > 3)
    {
        range_circles.erase(range_circles.begin());
        // net_vector_mag.erase(net_vector_mag.begin());
        solutions.erase(solutions.begin());
    }

    // Movement vectors in between each range circles
    if (movement_vectors.size() > 2)
    {
        movement_vectors.erase(movement_vectors.begin());
    }

    // [A11, A12] = vectorInCircles(movement vector of A, distance from investigate vehicle to range circle 1 , B1_true, D2, B2_true)

    // investigation_vector = 0;

    for (int i = 0; i < range_circles.size() - 1; i++)
    {

        // explore_vector = movement_vector.at(i);
        // net_vector = explore_vector - investigation_vector;
        net_vector.push_back(movement_vectors.at(i));
        // net_vector_mag.push_back(sqrt(pow(net_vector.at(0), 2) + pow(net_vector.at(1), 2)/*, pow(net_vector.at(2), 2)*/));

        double d1 = range_circles.at(i);
        double d2 = range_circles.at(i + 1);

        solutions.push_back(vectorLocalisation(net_vector, d1, d2));
    }

    // std::vector<std::vector<double>> movement_vectors;
    // movement_vectors.clear();

    double lowest_difference = 0;
    std::vector<double> lowest_index = {0,0};

    for (int i = 0; i < range_circles.size() - 1; i++)
    {
        for (int j = 0; j < 1; j++)
        {

            // diffs(i,j) = norm((A1(i,:) + A1_A2) - A2(j,:))
            std::vector<double> vector_x = solutions.at(0).at(i).at(0) + solutions.at(1).at(j).at(0) + net_vector.at(0);
            std::vector<double> vector_y = solutions.at(0).at(i).at(1) + solutions.at(1).at(j).at(1) + net_vector.at(1);

            difference = sqrt(pow(vector_x,2)+pow(vector_y,2));
            

            if (difference<lowest_difference || lowest_difference = 0)
            {
                    lowest_difference = difference;
                    lowest_index = {i,j};
            }

        }

    }

    std::vector <double> solution = solutions.at(lowest_index(0)).at(lowest_index(1));
    net_vector.clear();
    // v = v - (b2 - b1);
    // theta = acos((d1 ^ 2 + norm(v) ^ 2 - d2 ^ 2) / (2 * d1 * norm(v)));
    // vector_angle = atan2(v(2), v(1));
    // [ x1, y1 ] = pol2cart(theta + vector_angle, d1);
    // [ x2, y2 ] = pol2cart(-theta + vector_angle, d1);
    // solution1 = [ -x1, -y1 ] + b1; % first solution for A1
    // solution2 = [-x2, -y2] + b1; % second solution for A1
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