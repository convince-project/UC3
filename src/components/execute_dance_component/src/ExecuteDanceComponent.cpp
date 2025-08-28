/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "ExecuteDanceComponent.h"

#include <fstream>                      // per leggere i file JSON
#include <nlohmann/json.hpp>            // parsing JSON (nlohmann)
#include <cmath>                        // funzioni matematiche (atan2, M_PI, ecc.)
#include <map>
#include <sstream>                      // per costruire debug string
#include <cstdlib>                      // Per std::system (avvio controller esterni)
#include <thread>                       // Per sleep_for, thread management
#include <Eigen/Dense>                  // algebra lineare per rotazioni/vettori
#include <Eigen/Geometry>               // Quaternion / AngleAxis
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // updated header

// =============================================================================
// start()
// - inizializza ROS2 (se necessario), avvia/connette controller YARP,
// - apre porte YARP per ActionPlayer e per i client cartesiani,
// - carica coordinate artworks/POI,
// - registra servizi ROS2.
// Perché:
// - centralizza tutta la logica di inizializzazione e connessione necessaria
//   prima di iniziare a ricevere richieste di ExecuteDance.
// =============================================================================
bool ExecuteDanceComponent::start(int argc, char*argv[])
{
    // Se ROS2 non è inizializzato, inizializziamo
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ argc, /*argv*/ argv);
    }

    // Percorsi dei server cartesiani attesi
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";

    // (rimuoviamo il limite di tentativi: aspettiamo indefinitamente che i server RPC si registrino)

    // 1) Avvio del processo cartesian controller se non è già attivo.
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

    // Avvio del controller destro (stesso ragionamento del blocco precedente)
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
    
    // 2) Attesa sincrona senza timeout: aspettiamo che i server RPC si registrino al NameServer.
    //    Perché: vogliamo evitare che il processo termini se il controller impiega più tempo a partire.
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attesa della porta %s...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Ancora in attesa di %s...", cartesianPortLeft.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "La porta %s è ora disponibile.", cartesianPortLeft.c_str());

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attesa della porta %s...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Ancora in attesa di %s...", cartesianPortRight.c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "La porta %s è ora disponibile.", cartesianPortRight.c_str());

    // Inizializza connessione YARP ActionPlayer (client port)
    // Perché: ActionPlayer è la parte che riproduce sequenze (gesti) associate al parlato.
    yAPClientPortName = "/ExecuteDanceComponent/yarpActionsPlayerClient/rpc";
    
    // Pulizia: se una porta con lo stesso nome esiste, proviamo a disconnetterla prima di aprirne una nuova.
    if (yarp::os::Network::exists(yAPClientPortName)) {
        yWarning() << "Port" << yAPClientPortName << "already exists, attempting cleanup";
        // Proviamo a scollegare connessioni precedenti (tentativo di recupero)
        yarp::os::Network::disconnect(yAPClientPortName, "/yarpActionsPlayer/rpc");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Apertura porta client per ActionPlayer
    bool b = m_yAPClientPort.open(yAPClientPortName);
    if (!b)
    {
        yError() << "Cannot open yarpActionsPlayer client port";
        return false;
    }
    // Connetti client locale alla porta del server ActionPlayer
    yarp::os::Network::connect(yAPClientPortName, "/yarpActionsPlayer/rpc");
    
    // Creazione nodo ROS2 usato per servizi e subscription
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");

    // Carica coordinate delle opere da file JSON
    // Perché: le coordinate servono per il pointing; per ora usiamo file di configurazione.
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 4) Apri porte client YARP verso i server cartesian controller (left e right)
    //    Perché: nelle specifiche il controller cartesiano espone RPC per comandi come get_pose, go_to_pose, is_pose_reachable.
    {
        // Left arm controller client port: nome locale e connetti al server di riferimento  
        m_cartesianPortNameLeft = "/ExecuteDanceComponent/cartesianClientLeft/rpc";
        const std::string cartesianServerPortLeft = "/r1-cartesian-control/left_arm/rpc:i";

        // Pulizia eventuale porta esistente
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

        // Right arm controller client port (stesso pattern)
        m_cartesianPortNameRight = "/ExecuteDanceComponent/cartesianClientRight/rpc";
        const std::string cartesianServerPortRight = "/r1-cartesian-control/right_arm/rpc:i";

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
            // Avvia il task (esecuzione della danza/pointing) in thread separato
            this->executeTask(request);
            // Risposta immediata di accettazione della richiesta
            response->is_ok = true;
            response->error_msg = "";
        }
    );

    return true;
}

// =============================================================================
// close()
// - chiude risorse YARP e ferma ROS2.
// Perché:
// - evitare leak, liberare porte e driver al termine dell'esecuzione.
// =============================================================================
bool ExecuteDanceComponent::close()
{
    // Chiude il PolyDriver (se usato)
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    
    // Chiude le porte client cartesiane aperte in start()
    if (m_cartesianPortLeft.isOpen()) {
        m_cartesianPortLeft.close();
    }
    if (m_cartesianPortRight.isOpen()) {
        m_cartesianPortRight.close();
    }
    
    // Chiude ROS2 (shutdown)
    rclcpp::shutdown();  
    return true;
}

void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);  
}

// =============================================================================
// executeTask(request)
// - Funzione principale che esegue il comportamento richiesto dal servizio
// - Per ogni richiesta: recupera posizione dell'opera, ottiene posa della mano
//   dal controller cartesiano, calcola orientamento target e invia comandi
//   di reachability / go_to_pose o relativeMove.
// Perché:
// - centralizza il flusso: read pose -> compute orientation -> check reachability -> move
// - log dettagliati aiutano debug in simulazione/robot reale.
// =============================================================================
void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    RCLCPP_INFO(m_node->get_logger(), "EXECUTE TASK: starting executeTask for '%s'", request->dance_name.c_str());
    auto artworkIt = m_artworkCoords.find(request->dance_name);
    if (artworkIt == m_artworkCoords.end()) {
        RCLCPP_WARN(m_node->get_logger(), "No ARTWORK found '%s', aborting executeTask.", request->dance_name.c_str());
        return;
    }

    const double artwork_x = artworkIt->second[0];
    const double artwork_y = artworkIt->second[1];
    const double artwork_z = artworkIt->second[2];
    RCLCPP_INFO(m_node->get_logger(), "ARTWORK coords: '%s' [%.3f, %.3f, %.3f]",
                request->dance_name.c_str(), artwork_x, artwork_y, artwork_z);

    const Eigen::Vector3d artwork_pos(artwork_x, artwork_y, artwork_z);

    // Pre-scan: popola armDistances e cachedPoseValues
    std::vector<std::pair<std::string,double>> armDistances;
    std::map<std::string, std::vector<double>> cachedPoseValues;
    if (!preScanArticulatedArms(artwork_pos, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No available arms to perform pointing. Aborting executeTask.");
        return;
    }

    // Provo prima il braccio più vicino
    std::sort(armDistances.begin(), armDistances.end(),
              [](const auto &a, const auto &b){ return a.second < b.second; });

    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;
        RCLCPP_INFO(m_node->get_logger(), "ARM LOOP: trying arm '%s' (closest first)", armName.c_str());

        // seleziona porta attiva
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                           : "/r1-cartesian-control/right_arm/rpc:i";
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready at execution time, skipping", armName.c_str());
            continue;
        }

        auto it_cached = cachedPoseValues.find(armName);
        if (it_cached == cachedPoseValues.end()) {
            RCLCPP_WARN(m_node->get_logger(), "No cached pose for %s, skipping", armName.c_str());
            continue;
        }

        // compute orientation and related values
        Eigen::Vector3d hand_pos, vec_to_artwork;
        double vec_norm = 0.0;
        Eigen::Matrix3d R_hand;
        Eigen::Quaterniond q_target;
        if (!computeOrientationFromFlatPose(it_cached->second, artwork_pos, hand_pos, vec_to_artwork, vec_norm, R_hand, q_target)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: failed to compute orientation, skipping", armName.c_str());
            continue;
        }

        // debug logs
        Eigen::Quaterniond q_current(R_hand);
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: q_current (hand) = [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                    armName.c_str(), q_current.x(), q_current.y(), q_current.z(), q_current.w());
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: q_target (desired) = [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                    armName.c_str(), q_target.x(), q_target.y(), q_target.z(), q_target.w());
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: hand_pos = [%.3f, %.3f, %.3f], artwork_pos = [%.3f, %.3f, %.3f], vec_norm=%.6f",
                    armName.c_str(), hand_pos.x(), hand_pos.y(), hand_pos.z(), artwork_pos.x(), artwork_pos.y(), artwork_pos.z(), vec_norm);

        // quick check: artwork pose reachable with q_target?
        if (!isPoseReachable(activePort, artwork_pos, q_target)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: target artwork pose not reachable, trying next arm", armName.c_str());
            continue;
        }

        // probe binary search to get best candidate
        Eigen::Vector3d best_candidate;
        if (!probeBinarySearch(activePort, hand_pos, artwork_pos, vec_to_artwork, vec_norm, q_target, best_candidate)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: probing failed, trying next arm", armName.c_str());
            continue;
        }

        RCLCPP_INFO(m_node->get_logger(), "ARM %s: selected candidate pos=[%.3f, %.3f, %.3f] ; q_target=[%.6f, %.6f, %.6f, %.6f]",
                    armName.c_str(), best_candidate.x(), best_candidate.y(), best_candidate.z(),
                    q_target.x(), q_target.y(), q_target.z(), q_target.w());

        // final go_to_pose
        yarp::os::Bottle cmd_pose_final, res_pose_final;
        cmd_pose_final.addString("go_to_pose");
        cmd_pose_final.addFloat64(best_candidate.x()); cmd_pose_final.addFloat64(best_candidate.y()); cmd_pose_final.addFloat64(best_candidate.z());
        cmd_pose_final.addFloat64(q_target.x()); cmd_pose_final.addFloat64(q_target.y()); cmd_pose_final.addFloat64(q_target.z()); cmd_pose_final.addFloat64(q_target.w());
        cmd_pose_final.addFloat64(15.0);
        bool ok_pose_final = activePort->write(cmd_pose_final, res_pose_final);
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: go_to_pose (final candidate) write_ok=%d res_size=%ld", armName.c_str(), ok_pose_final, res_pose_final.size());
        if (ok_pose_final && res_pose_final.size() > 0 && res_pose_final.get(0).asVocab32() == yarp::os::createVocab32('o','k')) {
            RCLCPP_INFO(m_node->get_logger(), "POSITIONING SUCCESS: %s hand aligned toward artwork [%s]", armName.c_str(), request->dance_name.c_str());
            break;
        } else {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed for final candidate, trying next arm", armName.c_str());
            continue;
        }
    } // end for arms

    RCLCPP_INFO(m_node->get_logger(), "executeTask finished for '%s'", request->dance_name.c_str());
} 

// =============================================================================
// checkPoseReachabilityForArm()
// - Wrapper che invia is_pose_reachable al controller per un braccio specifico.
// Perché:
// - espone un metodo riusabile per testare se una posizione è raggiungibile prima di muoversi.
// =============================================================================
bool ExecuteDanceComponent::checkPoseReachabilityForArm(double x, double y, double z, const std::string& armName)
{
    RCLCPP_INFO(m_node->get_logger(),
               "REACHABILITY CHECK: Testing position [%.2f, %.2f, %.2f] for %s arm", 
               x, y, z, armName.c_str());

    yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;

    // Costruiamo la Bottle con is_pose_reachable + target pose (qui orientamento neutro)
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

    // Interpretiamo la risposta in base a vocab 'ok' convention
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

// =============================================================================
// sendPositionCommand()
// - Invia go_to_position (primitive semplificata) al controller per un braccio specifico.
// Perché:
// - comodo helper per invii semplici di posizione con durata fissa.
// =============================================================================
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

// =============================================================================
// checkPoseReachability()
// - Compatibilità legacy: reindirizza al controllo per il braccio sinistro
// Perché:
// - mantiene l'interfaccia precedente che altri moduli possono aspettarsi.
// =============================================================================
bool ExecuteDanceComponent::checkPoseReachability(double x, double y, double z)
{
    // Usa la nuova implementazione per il braccio sinistro come default
    return checkPoseReachabilityForArm(x, y, z, "LEFT");
}

// =============================================================================
// loadArtworkCoordinates()
// - Legge il file JSON con le coordinate absolute (map frame) delle opere.
// - Ritorna una mappa name -> [x,y,z].
// Perché:
// - separate storage dei POI/artwork permette test rapidi e modifica senza ricompilare.
// =============================================================================
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
        
        // Carichiamo ogni voce "artworks" come x,y,z assoluti nel frame della mappa
        for (auto& [name, data] : config.at("artworks").items()) {
            double x = data.at("x").get<double>();  // X nella mappa (metri)
            double y = data.at("y").get<double>();  // Y nella mappa (metri)
            double z = data.at("z").get<double>();  // Altezza Z dell'opera
            
            // Memorizziamo come vettore per essere usato dal controller cartesiano
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

bool ExecuteDanceComponent::transformPointMapToRobot(
    const geometry_msgs::msg::Point& map_point,
    geometry_msgs::msg::Point& out_robot_point,
    const std::string& robot_frame,
    double timeout_sec)
{
    // Trasforma map_point (frame "map") nel frame robot_frame usando TF2.
    // Ritorna true se la trasformazione ha successo e scrive il risultato in out_robot_point.
    // robot_frame es.: "base_link" o "base_footprint".
    // timeout_sec: tempo (s) da attendere per il transform prima di fallire.

    // Buffer + listener locali (ok se usati immediatamente)
    tf2_ros::Buffer tfBuffer(m_node->get_clock());
    tf2_ros::TransformListener tfListener(tfBuffer);

    const std::string map_frame = "map";
    const auto timeout_ms = std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000.0));

    // Attendi che la trasformazione sia disponibile (al più timeout_sec)
    if (!tfBuffer.canTransform(robot_frame, map_frame, rclcpp::Time(0), timeout_ms)) {
        RCLCPP_WARN(m_node->get_logger(),
                    "TF: transform %s <- %s not available within %.3fs",
                    robot_frame.c_str(), map_frame.c_str(), timeout_sec);
        return false;
    }

    try {
        // Ottieni l'ultima trasformazione map -> robot_frame
        geometry_msgs::msg::TransformStamped tf_stamped =
            tfBuffer.lookupTransform(robot_frame, map_frame, rclcpp::Time(0));

        // Costruiamo PointStamped in frame "map" e applichiamo la trasformazione
        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header.stamp = tf_stamped.header.stamp;
        p_in.header.frame_id = map_frame;
        p_in.point = map_point;

        tf2::doTransform(p_in, p_out, tf_stamped);

        out_robot_point = p_out.point;
        return true;
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF exception while transforming point: %s", ex.what());
        return false;
    }
}

// =============================================================================
// ExecuteDanceComponent
// =============================================================================
bool ExecuteDanceComponent::preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                                   std::vector<std::pair<std::string,double>>& armDistances,
                                                   std::map<std::string, std::vector<double>>& cachedPoseValues)
{
    armDistances.clear();
    cachedPoseValues.clear();

    for (const std::string armId : {"LEFT", "RIGHT"}) {
        yarp::os::Port* clientPort = (armId == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string serverPort = (armId == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                        : "/r1-cartesian-control/right_arm/rpc:i";

        if (!clientPort->isOpen() || !yarp::os::Network::isConnected(clientPort->getName(), serverPort)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not ready, skipping", armId.c_str());
            continue;
        }

        yarp::os::Bottle cmd_get, res_get;
        cmd_get.addString("get_pose");
        bool write_ok = clientPort->write(cmd_get, res_get);
        RCLCPP_INFO(m_node->get_logger(), "PRE-SCAN: %s get_pose write_ok=%d res_size=%ld", armId.c_str(), write_ok, res_get.size());
        if (!write_ok) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", armId.c_str());
            continue;
        }

        std::vector<double> flat_pose;
        for (size_t i = 0; i < res_get.size(); ++i) {
            if (res_get.get(i).isList()) {
                yarp::os::Bottle* sub = res_get.get(i).asList();
                for (size_t j = 0; j < sub->size(); ++j) flat_pose.push_back(sub->get(j).asFloat64());
            } else {
                flat_pose.push_back(res_get.get(i).asFloat64());
            }
        }

        if (flat_pose.size() != 18 && flat_pose.size() != 16) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: unexpected pose size=%zu for %s, skipping", flat_pose.size(), armId.c_str());
            continue;
        }

        const size_t pose_offset = (flat_pose.size() == 18) ? 2u : 0u;
        const double hand_tx = flat_pose[pose_offset + 3];
        const double hand_ty = flat_pose[pose_offset + 7];
        const double hand_tz = flat_pose[pose_offset + 11];

        const double euclidean_dist = std::hypot(std::hypot(artwork_pos.x() - hand_tx, artwork_pos.y() - hand_ty), artwork_pos.z() - hand_tz);
        RCLCPP_INFO(m_node->get_logger(), "PRE-SCAN: %s hand pos=[%.3f, %.3f, %.3f] dist=%.3f",
                    armId.c_str(), hand_tx, hand_ty, hand_tz, euclidean_dist);

        armDistances.emplace_back(armId, euclidean_dist);
        cachedPoseValues.emplace(armId, std::move(flat_pose));
    }

    return !armDistances.empty();
}

bool ExecuteDanceComponent::computeOrientationFromFlatPose(const std::vector<double>& flat_pose,
                                                           const Eigen::Vector3d& artwork_pos,
                                                           Eigen::Vector3d& hand_pos,
                                                           Eigen::Vector3d& vec_to_artwork,
                                                           double& vec_norm,
                                                           Eigen::Matrix3d& R_hand,
                                                           Eigen::Quaterniond& q_target)
{
    const size_t pose_offset = (flat_pose.size() == 18) ? 2u : 0u;
    if (flat_pose.size() < pose_offset + 12) return false;

    const double hand_x = flat_pose[pose_offset + 3];
    const double hand_y = flat_pose[pose_offset + 7];
    const double hand_z = flat_pose[pose_offset + 11];
    hand_pos = Eigen::Vector3d(hand_x, hand_y, hand_z);

    Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>> T_row(flat_pose.data() + pose_offset);
    R_hand = T_row.topLeftCorner<3,3>();

    vec_to_artwork = artwork_pos - hand_pos;
    vec_norm = vec_to_artwork.norm();
    if (vec_norm <= 1e-9) return false;
    vec_to_artwork.normalize();

    const Eigen::Vector3d hand_local_x = R_hand.col(0);
    const double cos_angle = std::clamp(hand_local_x.dot(vec_to_artwork), -1.0, 1.0);
    Eigen::Vector3d rotation_axis = hand_local_x.cross(vec_to_artwork);
    double axis_norm = rotation_axis.norm();

    if (axis_norm <= 1e-6) {
        if (cos_angle > 0.999999) {
            q_target = Eigen::Quaterniond(R_hand);
        } else {
            Eigen::Vector3d fallback = (std::abs(hand_local_x.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
            rotation_axis = hand_local_x.cross(fallback);
            if (rotation_axis.norm() <= 1e-9) rotation_axis = Eigen::Vector3d::UnitZ();
            rotation_axis.normalize();
            Eigen::AngleAxisd aa(M_PI, rotation_axis);
            Eigen::Matrix3d R_target = aa * R_hand;
            q_target = Eigen::Quaterniond(R_target);
        }
    } else {
        rotation_axis.normalize();
        double angle = std::acos(cos_angle);
        Eigen::AngleAxisd aa(angle, rotation_axis);
        Eigen::Matrix3d R_target = aa * R_hand;
        q_target = Eigen::Quaterniond(R_target);
    }
    return true;
}

bool ExecuteDanceComponent::isPoseReachable(yarp::os::Port* activePort,
                                            const Eigen::Vector3d& candidate,
                                            const Eigen::Quaterniond& q_target)
{
    yarp::os::Bottle cmd_check_local, res_check_local;
    cmd_check_local.addString("is_pose_reachable");
    cmd_check_local.addFloat64(candidate.x());
    cmd_check_local.addFloat64(candidate.y());
    cmd_check_local.addFloat64(candidate.z());
    cmd_check_local.addFloat64(q_target.x());
    cmd_check_local.addFloat64(q_target.y());
    cmd_check_local.addFloat64(q_target.z());
    cmd_check_local.addFloat64(q_target.w());
    bool ok_local = activePort->write(cmd_check_local, res_check_local);
    if (!ok_local || res_check_local.size() == 0) {
        RCLCPP_WARN(m_node->get_logger(), "RPC error on is_pose_reachable for candidate [%.3f,%.3f,%.3f]",
                    candidate.x(), candidate.y(), candidate.z());
        return false;
    }
    return res_check_local.get(0).asVocab32() == yarp::os::createVocab32('o','k');
}

bool ExecuteDanceComponent::probeBinarySearch(yarp::os::Port* activePort,
                                              const Eigen::Vector3d& hand_pos,
                                              const Eigen::Vector3d& artwork_pos,
                                              const Eigen::Vector3d& vec_to_artwork,
                                              double vec_norm,
                                              const Eigen::Quaterniond& q_target,
                                              Eigen::Vector3d& out_best_candidate)
{
    (void)artwork_pos; // suppress unused-parameter warning

    const double safety_margin = 0.05;
    const double pos_tol = 1e-3;
    const int max_iters = 20;

    double max_t = std::max(0.0, vec_norm - safety_margin);
    if (max_t <= 0.0) max_t = 0.0;

    Eigen::Vector3d candidate_max = hand_pos + vec_to_artwork * max_t;
    if (isPoseReachable(activePort, candidate_max, q_target)) {
        out_best_candidate = candidate_max;
        return true;
    }

    double lo = 0.0;
    double hi = max_t;
    double best_t = 0.0;
    for (int it = 0; it < max_iters && (hi - lo) > pos_tol; ++it) {
        double mid = 0.5 * (lo + hi);
        Eigen::Vector3d candidate = hand_pos + vec_to_artwork * mid;
        RCLCPP_DEBUG(m_node->get_logger(), "binary iter=%d mid=%.6f candidate=[%.3f,%.3f,%.3f]",
                     it, mid, candidate.x(), candidate.y(), candidate.z());
        if (isPoseReachable(activePort, candidate, q_target)) {
            best_t = mid;
            lo = mid;
        } else {
            hi = mid;
        }
    }
    out_best_candidate = hand_pos + vec_to_artwork * best_t;
    return true;
}

