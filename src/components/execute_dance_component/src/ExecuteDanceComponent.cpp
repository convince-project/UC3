/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#include "ExecuteDanceComponent.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <cmath>
#include <map>
#include <sstream>
#include <cstdlib> // Per std::system
#include <thread> // Per sleep_for
#include <Eigen/Dense>
#include <Eigen/Geometry>

bool ExecuteDanceComponent::start(int argc, char*argv[])
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";
    const int max_retries = 10;

    // 1. Avvia il cartesian controller se non è già attivo
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avvio r1-cartesian-control LEFT...");
        std::string cmd = "r1-cartesian-control --from " + cartesianControllerIniPathLeft + " > /dev/null 2>&1 &";
        int ret = std::system(cmd.c_str());
        if (ret == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nell'avvio di r1-cartesian-control");
            return false;
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r1-cartesian-control per braccio sinistro avviato con successo.");
        }
    }

    // Avvia il controller destro
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avvio r1-cartesian-control RIGHT...");
        std::string cmd = "r1-cartesian-control --from " + cartesianControllerIniPathRight + " > /dev/null 2>&1 &";
        int ret = std::system(cmd.c_str());
        if (ret == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nell'avvio di r1-cartesian-control");
            return false;
        }
        else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r1-cartesian-control per braccio destro avviato con successo.");
        }
    }
    
    // 2. Attendi che la porta sia disponibile
    int retries = max_retries;
    while (!yarp::os::Network::exists(cartesianPortLeft) && retries-- > 0) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attendo che la porta %s sia disponibile...", cartesianPortLeft.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "La porta %s non è disponibile dopo %d secondi.", cartesianPortLeft.c_str(), max_retries);
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "r1-cartesian-control attivo e porta disponibile.");
    }

    // Initialize YarpActionPlayer connection 
    yAPClientPortName = "/ExecuteDanceComponent/yarpActionsPlayerClient/rpc";
    
    // Clean up any existing port with same name
    if (yarp::os::Network::exists(yAPClientPortName)) {
        yWarning() << "Port" << yAPClientPortName << "already exists, attempting cleanup";
        // Force disconnect any existing connections
        yarp::os::Network::disconnect(yAPClientPortName, "/yarpActionsPlayer/rpc");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    bool b = m_yAPClientPort.open(yAPClientPortName);
    if (!b)
    {
        yError() << "Cannot open yarpActionsPlayer client port";
        return false;
    }
    yarp::os::Network::connect(yAPClientPortName, "/yarpActionsPlayer/rpc");
    
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");

    // // 1) subscribe to /amcl_pose
    // m_amclSub = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    //     "/amcl_pose", 10,
    //     std::bind(&ExecuteDanceComponent::amclPoseCallback, this, std::placeholders::_1));
    
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 4) open YARP CartesianController ports for both arms
    {
        // Left arm controller
        m_cartesianPortNameLeft = "/ExecuteDanceComponent/cartesianClientLeft/rpc";
        const std::string cartesianServerPortLeft = "/r1-cartesian-control/left_arm/rpc:i";

        // Clean up any existing port with same name
        if (yarp::os::Network::exists(m_cartesianPortNameLeft)) {
            yWarning() << "Port" << m_cartesianPortNameLeft << " already exists, attempting cleanup";
            yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianServerPortLeft);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        bool okLeft = m_cartesianPortLeft.open(m_cartesianPortNameLeft);
        if (!okLeft) {
            yError() << "Cannot open Left CartesianController client port";
            return false;
        }
        yarp::os::Network::connect(m_cartesianPortNameLeft, cartesianServerPortLeft);
        RCLCPP_INFO(m_node->get_logger(), "Connected to LEFT CartesianController via YARP port.");

        // Right arm controller
        m_cartesianPortNameRight = "/ExecuteDanceComponent/cartesianClientRight/rpc";
        const std::string cartesianServerPortRight = "/r1-cartesian-control/right_arm/rpc:i";

        // Clean up any existing port with same name
        if (yarp::os::Network::exists(m_cartesianPortNameRight)) {
            yWarning() << "Port" << m_cartesianPortNameRight << " already exists, attempting cleanup";
            yarp::os::Network::disconnect(m_cartesianPortNameRight, cartesianServerPortRight);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        bool okRight = m_cartesianPortRight.open(m_cartesianPortNameRight);
        if (!okRight) {
            yError() << "Cannot open Right CartesianController client port";
            return false;
        }
        yarp::os::Network::connect(m_cartesianPortNameRight, cartesianServerPortRight);
        RCLCPP_INFO(m_node->get_logger(), "Connected to RIGHT CartesianController via YARP port.");
    }

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteDanceComponent::start");      

    // // Test automatico: chiama il servizio ExecuteDance con "inizio" dopo l'avvio
    // {
    //     auto client = m_node->create_client<execute_dance_interfaces::srv::ExecuteDance>("/ExecuteDanceComponent/ExecuteDance");
    //     // Attendi che il servizio sia disponibile
    //     while (!client->wait_for_service(std::chrono::seconds(1))) {
    //         RCLCPP_INFO(m_node->get_logger(), "Waiting for /ExecuteDanceComponent/ExecuteDance service...");
    //     }
    //     auto request = std::make_shared<execute_dance_interfaces::srv::ExecuteDance::Request>();
    //     request->dance_name = "inizio";
    //     RCLCPP_INFO(m_node->get_logger(), "Calling ExecuteDance service with dance_name='inizio'");
    //     auto result = client->async_send_request(request);
    //     // Attendi la risposta (sincrono per debug)
    //     if (rclcpp::spin_until_future_complete(m_node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    //         auto response = result.get();
    //         RCLCPP_INFO(m_node->get_logger(), "Service response: is_ok=%d, error_msg=%s", response->is_ok, response->error_msg.c_str());
    //     } else {
    //         RCLCPP_ERROR(m_node->get_logger(), "Failed to call ExecuteDance service");
    //     }
    // }

    m_executeDanceService = m_node->create_service<execute_dance_interfaces::srv::ExecuteDance>(
        "/ExecuteDanceComponent/ExecuteDance",
        [this](
            const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
            std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response)
        {
            this->executeTask(request);
            response->is_ok = true;
            response->error_msg = "";
        }
    );

    return true;
}

bool ExecuteDanceComponent::close()
{
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    
    // Close both cartesian ports
    if (m_cartesianPortLeft.isOpen()) {
        m_cartesianPortLeft.close();
    }
    if (m_cartesianPortRight.isOpen()) {
        m_cartesianPortRight.close();
    }
    
    rclcpp::shutdown();  
    return true;
}

void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);  
}

void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    RCLCPP_INFO(m_node->get_logger(), "DEBUG: Requested dance_name: '%s'", request->dance_name.c_str());

    auto artworkIt = m_artworkCoords.find(request->dance_name);
    if (artworkIt == m_artworkCoords.end()) {
        RCLCPP_WARN(m_node->get_logger(),
                    "No ARTWORK found '%s', skipping positioning.",
                    request->dance_name.c_str());
        return;
    }

    double artwork_x = artworkIt->second[0];
    double artwork_y = artworkIt->second[1];
    double artwork_z = artworkIt->second[2];

    // Per ogni mano (LEFT, RIGHT)
    for (const std::string armName : {"LEFT", "RIGHT"}) {
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;

        // 1. Verifica che la porta sia aperta e connessa
        if (!activePort->isOpen()) {
            RCLCPP_ERROR(m_node->get_logger(),
                "YARP port for %s arm is not open!", armName.c_str());
            continue;
        }
        std::string remotePort = (armName == "LEFT")
            ? "/r1-cartesian-control/left_arm/rpc:i"
            : "/r1-cartesian-control/right_arm/rpc:i";
        if (!yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_ERROR(m_node->get_logger(),
                "YARP port %s is not connected to %s!", activePort->getName().c_str(), remotePort.c_str());
            continue;
        }

        // 2. Ottieni la posa attuale della mano
        yarp::os::Bottle cmd_get, res_get;
        cmd_get.addString("get_pose");
        bool write_ok = activePort->write(cmd_get, res_get);

        // Logga la risposta grezza
        std::ostringstream oss;
        for (size_t i = 0; i < res_get.size(); ++i) {
            oss << res_get.get(i).toString() << " ";
        }
        RCLCPP_DEBUG(m_node->get_logger(),
            "get_pose response from %s: write_ok=%d, size=%ld, content=[%s]",
            armName.c_str(), write_ok, res_get.size(), oss.str().c_str());

        if (!write_ok) {
            RCLCPP_ERROR(m_node->get_logger(),
                "GET_POSE ERROR: write() failed for %s arm (artwork: '%s' at [%.2f, %.2f, %.2f])",
                armName.c_str(), request->dance_name.c_str(), artwork_x, artwork_y, artwork_z);
            continue;
        }

        // dopo aver ricevuto res_get...

        // Appiattisci la risposta se contiene liste annidate
        std::vector<double> pose_values;
        for (size_t i = 0; i < res_get.size(); ++i) {
            if (res_get.get(i).isList()) {
                yarp::os::Bottle* sub = res_get.get(i).asList();
                for (size_t j = 0; j < sub->size(); ++j) {
                    pose_values.push_back(sub->get(j).asFloat64());
                }
            } else {
                pose_values.push_back(res_get.get(i).asFloat64());
            }
        }

        if (pose_values.size() != 18 && pose_values.size() != 16) {
            RCLCPP_ERROR(m_node->get_logger(),
                "GET_POSE ERROR: Unexpected total value count (%zu) from %s arm. Content: [%s]",
                pose_values.size(), armName.c_str(), oss.str().c_str());
            continue;
        }

        size_t offset = (pose_values.size() == 18) ? 2 : 0;

        // Parsing posizione e rotazione
        double hand_x = pose_values[offset + 3];   // m03
        double hand_y = pose_values[offset + 7];   // m13
        double hand_z = pose_values[offset + 11];  // m23

        RCLCPP_INFO(m_node->get_logger(),
            "%s arm: START pose [x=%.3f, y=%.3f, z=%.3f] -> TARGET artwork '%s' [x=%.3f, y=%.3f, z=%.3f]",
            armName.c_str(), hand_x, hand_y, hand_z,
            request->dance_name.c_str(), artwork_x, artwork_y, artwork_z);

        Eigen::Matrix3d R_hand;
        R_hand << pose_values[offset + 0], pose_values[offset + 1], pose_values[offset + 2],
                  pose_values[offset + 4], pose_values[offset + 5], pose_values[offset + 6],
                  pose_values[offset + 8], pose_values[offset + 9], pose_values[offset + 10];

        // 2. Calcola la direzione artwork-mano e l'orientamento target lungo X
        Eigen::Vector3d hand_pos(hand_x, hand_y, hand_z);
        Eigen::Vector3d artwork_pos(artwork_x, artwork_y, artwork_z);
        Eigen::Vector3d vec_hand_to_artwork = (artwork_pos - hand_pos).normalized();

        // Orientamento attuale X della mano (prima colonna della matrice di rotazione)
        Eigen::Vector3d hand_x_axis = R_hand.col(0);

        // Angolo tra X della mano e direzione verso l'artwork PRIMA della rotazione
        double cos_angle_before = hand_x_axis.dot(vec_hand_to_artwork);
        cos_angle_before = std::clamp(cos_angle_before, -1.0, 1.0);
        double angle_before_rad = std::acos(cos_angle_before);
        double angle_before_deg = angle_before_rad * 180.0 / M_PI;

        RCLCPP_INFO(m_node->get_logger(),
            "%s arm: ANGLE between hand X axis and artwork direction BEFORE rotation: %.2f deg",
            armName.c_str(), angle_before_deg);

        // Calcola angolo tra X della mano e direzione verso l'artwork (solo su X)
        double cos_angle = hand_x_axis.dot(vec_hand_to_artwork);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        double angle_x = std::acos(cos_angle);

        // Determina il verso della rotazione (asse Z della mano)
        Eigen::Vector3d axis = hand_x_axis.cross(vec_hand_to_artwork);
        double sign = (axis.z() >= 0) ? 1.0 : -1.0;
        angle_x *= sign;

        // 3. Crea la nuova orientazione ruotando attorno all'asse X della mano
        Eigen::AngleAxisd rot_x(angle_x, hand_x_axis);
        Eigen::Matrix3d R_target = rot_x * R_hand;

        // 4. Converti la rotazione in quaternion per go_to_pose
        Eigen::Quaterniond q_target(R_target);
        Eigen::Quaterniond q_hand(R_hand);

        // Calcola la differenza di orientamento (in gradi) tra la rotazione iniziale e quella target
        double dot = std::abs(q_hand.dot(q_target));
        dot = std::clamp(dot, -1.0, 1.0);
        double delta_angle_rad = 2.0 * std::acos(dot);
        double delta_angle_deg = delta_angle_rad * 180.0 / M_PI;

        RCLCPP_INFO(m_node->get_logger(),
            "%s arm: ORIENTATION DELTA = %.2f deg (from START to ROTATED TARGET)", armName.c_str(), delta_angle_deg);

        // Stampa la posa target dopo la rotazione
        RCLCPP_INFO(m_node->get_logger(),
            "%s arm: ROTATED TARGET pose [x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f] (aligned with artwork '%s')",
            armName.c_str(), hand_x, hand_y, hand_z,
            q_target.x(), q_target.y(), q_target.z(), q_target.w(),
            request->dance_name.c_str());

        // Orientamento X della mano DOPO la rotazione
        Eigen::Vector3d hand_x_axis_after = R_target.col(0);
        double cos_angle_after = hand_x_axis_after.dot(vec_hand_to_artwork);
        cos_angle_after = std::clamp(cos_angle_after, -1.0, 1.0);
        double angle_after_rad = std::acos(cos_angle_after);
        double angle_after_deg = angle_after_rad * 180.0 / M_PI;

        RCLCPP_INFO(m_node->get_logger(),
            "%s arm: ANGLE between hand X axis and artwork direction AFTER rotation: %.2f deg",
            armName.c_str(), angle_after_deg);

        // 5. Verifica raggiungibilità della nuova posa
        yarp::os::Bottle cmd_check, res_check;
        cmd_check.addString("is_pose_reachable");
        cmd_check.addFloat64(hand_x);
        cmd_check.addFloat64(hand_y);
        cmd_check.addFloat64(hand_z);
        cmd_check.addFloat64(q_target.x());
        cmd_check.addFloat64(q_target.y());
        cmd_check.addFloat64(q_target.z());
        cmd_check.addFloat64(q_target.w());
        if (!activePort->write(cmd_check, res_check) || res_check.size() == 0 || res_check.get(0).asVocab32() != yarp::os::createVocab32('o','k')) {
            RCLCPP_WARN(m_node->get_logger(),
                        "POSE NOT REACHABLE: %s hand cannot reach rotated pose at [%.2f, %.2f, %.2f]", armName.c_str(), hand_x, hand_y, hand_z);
            continue;
        }

        // 6. Invia comando go_to_pose
        yarp::os::Bottle cmd_pose, res_pose;
        cmd_pose.addString("go_to_pose");
        cmd_pose.addFloat64(hand_x);
        cmd_pose.addFloat64(hand_y);
        cmd_pose.addFloat64(hand_z);
        cmd_pose.addFloat64(q_target.x());
        cmd_pose.addFloat64(q_target.y());
        cmd_pose.addFloat64(q_target.z());
        cmd_pose.addFloat64(q_target.w());
        cmd_pose.addFloat64(15.0); // durata traiettoria

        if (!activePort->write(cmd_pose, res_pose) || res_pose.size() == 0 || res_pose.get(0).asVocab32() != yarp::os::createVocab32('o','k')) {
            RCLCPP_WARN(m_node->get_logger(),
                        "POSITIONING FAILED: %s hand could not move to rotated pose", armName.c_str());
        } else {
            RCLCPP_INFO(m_node->get_logger(),
                        "POSITIONING SUCCESS: %s hand moved to rotated pose aligned with artwork [%s]", armName.c_str(), request->dance_name.c_str());
        }
    }
}

// NUOVA FUNZIONE: Verifica raggiungibilità per un braccio specifico
bool ExecuteDanceComponent::checkPoseReachabilityForArm(double x, double y, double z, const std::string& armName)
{
    RCLCPP_INFO(m_node->get_logger(),
               "REACHABILITY CHECK: Testing position [%.2f, %.2f, %.2f] for %s arm", 
               x, y, z, armName.c_str());

    yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;

    // Usa l'API ufficiale is_pose_reachable
    yarp::os::Bottle cmd, res;
    cmd.addString("is_pose_reachable");
    cmd.addFloat64(x);     // Target X coordinate
    cmd.addFloat64(y);     // Target Y coordinate  
    cmd.addFloat64(z);     // Target Z coordinate
    cmd.addFloat64(0.0);   // Quaternion X (orientamento neutro)
    cmd.addFloat64(0.0);   // Quaternion Y (orientamento neutro)
    cmd.addFloat64(0.0);   // Quaternion Z (orientamento neutro)
    cmd.addFloat64(1.0);   // Quaternion W (orientamento neutro)

    bool ok = activePort->write(cmd, res);
    if (!ok || res.size() == 0) {
        RCLCPP_ERROR(m_node->get_logger(),
                    "REACHABILITY ERROR: Failed to communicate with %s CartesianController", armName.c_str());
        return false;
    }

    if (res.get(0).asVocab32() == yarp::os::createVocab32('o','k')) {
        RCLCPP_INFO(m_node->get_logger(),
                   "✅ REACHABILITY RESULT: Position [%.2f, %.2f, %.2f] is REACHABLE by %s arm",
                   x, y, z, armName.c_str());
        return true;
    } else {
        RCLCPP_WARN(m_node->get_logger(),
                   "❌ REACHABILITY RESULT: Position [%.2f, %.2f, %.2f] is NOT REACHABLE by %s arm: %s",
                   x, y, z, armName.c_str(), res.toString().c_str());
        return false;
    }
}

// NUOVA FUNZIONE: Invia comando di posizione a un braccio specifico
bool ExecuteDanceComponent::sendPositionCommand(double x, double y, double z, const std::string& armName)
{
    yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;

    yarp::os::Bottle cmd, res;
    cmd.addString("go_to_position");
    cmd.addFloat64(x);     // Posizione X target
    cmd.addFloat64(y);     // Posizione Y target  
    cmd.addFloat64(z);     // Posizione Z target
    cmd.addFloat64(30.0);  // Trajectory duration

    RCLCPP_INFO(m_node->get_logger(),
                "SENDING POSITIONING COMMAND: %s arm go_to_position [%.2f, %.2f, %.2f], duration=%.2f",
                armName.c_str(), x, y, z, 30.0);

    bool ok = activePort->write(cmd, res);
    if (!ok || res.size() == 0 || res.get(0).asVocab32() != yarp::os::createVocab32('o','k')) {
        RCLCPP_WARN(m_node->get_logger(),
                    "POSITIONING FAILED: %s arm did not accept go_to_position command: %s", 
                    armName.c_str(), res.toString().c_str());
        return false;
    } else {
        RCLCPP_INFO(m_node->get_logger(), 
                    "POSITIONING COMMAND SENT: %s arm accepted go_to_position command", 
                    armName.c_str());
        return true;
    }
}

// FUNZIONE LEGACY: Manteniamo per compatibilità
bool ExecuteDanceComponent::checkPoseReachability(double x, double y, double z)
{
    // Usa la nuova implementazione per il braccio sinistro come default
    return checkPoseReachabilityForArm(x, y, z, "LEFT");
}

std::map<std::string, std::vector<double>> ExecuteDanceComponent::loadArtworkCoordinates(const std::string& filename)
{
    std::map<std::string, std::vector<double>> artworkMap;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(m_node->get_logger(), "Unable to open artwork coordinates file: '%s'", filename.c_str());
            return artworkMap;
        }
        
        auto config = nlohmann::ordered_json::parse(file);
        
        // Load complete X,Y,Z coordinates for each artwork
        // Coordinates are in the global museum map reference frame
        for (auto& [name, data] : config.at("artworks").items()) {
            double x = data.at("x").get<double>();  // Absolute X position in map frame
            double y = data.at("y").get<double>();  // Absolute Y position in map frame  
            double z = data.at("z").get<double>();  // Z height of artwork (for precise pointing)
            
            // Store as vector [x, y, z] for use with CartesianControl
            artworkMap.emplace(name, std::vector<double>{x, y, z});
            
            RCLCPP_INFO(m_node->get_logger(), 
                       "Loaded artwork '%s' at coordinates [%.2f, %.2f, %.2f]", 
                       name.c_str(), x, y, z);
        }
        
        RCLCPP_INFO(m_node->get_logger(), "Loaded %zu artworks from '%s'", 
                   artworkMap.size(), filename.c_str());
        
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing artwork file '%s': %s", 
                    filename.c_str(), ex.what());
    }
    
    return artworkMap;
}

