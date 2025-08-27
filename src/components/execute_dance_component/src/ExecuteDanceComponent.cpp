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
    // Log iniziale: quale danza/opera è stata richiesta
    RCLCPP_INFO(m_node->get_logger(), "DEBUG: Requested dance_name: '%s'", request->dance_name.c_str());
    RCLCPP_INFO(m_node->get_logger(), "EXECUTE TASK: starting executeTask thread for '%s'", request->dance_name.c_str());

    // Recupera coordinate artwork; se non presente, abortiamo rapidamente.
    auto artworkIt = m_artworkCoords.find(request->dance_name);
    if (artworkIt == m_artworkCoords.end()) {
        RCLCPP_WARN(m_node->get_logger(),
                    "No ARTWORK found '%s', skipping positioning.",
                    request->dance_name.c_str());
        return;
    }

    // Estraggo coordinate (x,y,z) dell'opera
    double artwork_x = artworkIt->second[0];
    double artwork_y = artworkIt->second[1];
    double artwork_z = artworkIt->second[2];

    RCLCPP_INFO(m_node->get_logger(),
                "ARTWORK coords: name='%s' x=%.3f y=%.3f z=%.3f",
                request->dance_name.c_str(), artwork_x, artwork_y, artwork_z);

    // Ciclo sulle due mani: LEFT e RIGHT
    // Perché: il componente può puntare con entrambe le mani (configurabile dal comportamento).
    // --- PRE-SCAN: interroghiamo entrambe le mani per ottenere la posa corrente,
    // calcoliamo la distanza all'artwork e ordiniamo le mani per distanza crescente.
    std::vector<std::pair<std::string,double>> armDistances;
    std::map<std::string, std::vector<double>> cachedPoseValues;

    for (const std::string arm : {"LEFT", "RIGHT"}) {
        yarp::os::Port* port = (arm == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        std::string remotePort = (arm == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i" : "/r1-cartesian-control/right_arm/rpc:i";

        if (!port->isOpen()) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not open, skipping pre-scan for this arm", arm.c_str());
            continue;
        }
        if (!yarp::os::Network::isConnected(port->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not connected to %s, skipping", arm.c_str(), remotePort.c_str());
            continue;
        }

        yarp::os::Bottle cmd_get, res_get;
        cmd_get.addString("get_pose");
        bool ok = port->write(cmd_get, res_get);
        RCLCPP_INFO(m_node->get_logger(), "PRE-SCAN: %s get_pose write_ok=%d res_size=%ld", arm.c_str(), ok, res_get.size());
        if (!ok) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", arm.c_str());
            continue;
        }

        // parse response into vector<double>
        std::vector<double> pose_values;
        for (size_t i = 0; i < res_get.size(); ++i) {
            if (res_get.get(i).isList()) {
                yarp::os::Bottle* sub = res_get.get(i).asList();
                for (size_t j = 0; j < sub->size(); ++j) pose_values.push_back(sub->get(j).asFloat64());
            } else {
                pose_values.push_back(res_get.get(i).asFloat64());
            }
        }
        if (pose_values.size() != 18 && pose_values.size() != 16) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: unexpected pose size=%zu for %s, skipping", pose_values.size(), arm.c_str());
            continue;
        }

        size_t offset = (pose_values.size() == 18) ? 2 : 0;
        double hx = pose_values[offset + 3];
        double hy = pose_values[offset + 7];
        double hz = pose_values[offset + 11];
        double dist = std::hypot(std::hypot(artwork_x - hx, artwork_y - hy), artwork_z - hz);

        RCLCPP_INFO(m_node->get_logger(), "PRE-SCAN: %s hand pos=[%.3f,%.3f,%.3f] dist_to_artwork=%.3f", arm.c_str(), hx, hy, hz, dist);
        armDistances.emplace_back(arm, dist);
        cachedPoseValues.emplace(arm, pose_values);
    }

    if (armDistances.empty()) {
        RCLCPP_ERROR(m_node->get_logger(), "No available arms (ports) to perform pointing. Aborting executeTask.");
        return;
    }

    // ordina per distanza crescente (prima la mano più vicina)
    std::sort(armDistances.begin(), armDistances.end(), [](const auto &a, const auto &b){
        return a.second < b.second;
    });

    // ora proviamo le mani nell'ordine calcolato
    for (const auto &entry : armDistances) {
        const std::string armName = entry.first;
        RCLCPP_INFO(m_node->get_logger(), "ARM LOOP (ordered): trying arm '%s' (closer first)...", armName.c_str());

        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i" : "/r1-cartesian-control/right_arm/rpc:i";
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready at execution time, skipping", armName.c_str());
            continue;
        }

        // Recupera pose_values dalla cache (pre-scan)
        auto it_cached = cachedPoseValues.find(armName);
        if (it_cached == cachedPoseValues.end()) {
            RCLCPP_WARN(m_node->get_logger(), "No cached pose for %s, skipping", armName.c_str());
            continue;
        }
        std::vector<double> pose_values = it_cached->second;
        size_t offset = (pose_values.size() == 18) ? 2 : 0;
        double hand_x = pose_values[offset + 3];
        double hand_y = pose_values[offset + 7];
        double hand_z = pose_values[offset + 11];

        RCLCPP_INFO(m_node->get_logger(), "ARM %s: hand pos [x=%.3f y=%.3f z=%.3f] (using cached pre-scan)", armName.c_str(), hand_x, hand_y, hand_z);

        // costruiamo R_hand come prima
        // assume pose_values è std::vector<double> (o array) e contiene 16 valori di una 4x4
        // Proviamo il mapping row-major direttamente (M_row deve essere visibile nel scope seguente)
        Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>> M_row(pose_values.data() + offset);
            Eigen::Matrix3d R_hand_row = M_row.topLeftCorner<3,3>();

            // --- mapping alternativa (col-major) ---
            Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::ColMajor>> M_col(pose_values.data() + offset);
            Eigen::Matrix3d R_hand_col = M_col.topLeftCorner<3,3>();

            // --- debug: stampo entrambi in output per verificare quale è corretto ---
            // (sostituisci con il logger del tuo progetto se presente)
            std::cout << "Mapped (row-major) 4x4:\n" << M_row << "\n";
            std::cout << "Top-left 3x3 (row-major mapping):\n" << R_hand_row << "\n";
            std::cout << "Mapped (col-major) 4x4:\n" << M_col << "\n";
            std::cout << "Top-left 3x3 (col-major mapping):\n" << R_hand_col << "\n";

            // Scegli quello giusto per l'uso successivo:
            // Se i dati sono row-major, mantenere R_hand = R_hand_row;
            // altrimenti usare R_hand_col o trasporre:
            Eigen::Matrix3d R_hand = R_hand_row; // <-- modifica qui se necessario

            // calcolo direzione, axis-angle, check is_pose_reachable, go_to_pose...
            // Per evitare duplicazione qui riutilizziamo il blocco esistente invocando la stessa logica
            // (copia del flusso già presente sotto lo stesso file) — per brevità riusiamo lo stesso codice
            // che segue esattamente le stesse operazioni (calcoli e chiamate RPC) con hand_x/hand_y/hand_z e R_hand.

            // 2) Calcola la direzione dal hand all'artwork e normalizzala.
            Eigen::Vector3d hand_pos(hand_x, hand_y, hand_z);
            Eigen::Vector3d artwork_pos(artwork_x, artwork_y, artwork_z);
            Eigen::Vector3d vec_hand_to_artwork = (artwork_pos - hand_pos);
            double vec_norm = vec_hand_to_artwork.norm();
            if (vec_norm > 1e-9) vec_hand_to_artwork.normalize();
            else {
                RCLCPP_WARN(m_node->get_logger(), "ARM %s: degenerate vector from hand to artwork (norm=%.6f)", armName.c_str(), vec_norm);
                continue;
            }

            RCLCPP_INFO(m_node->get_logger(), "ARM %s: vec_hand_to_artwork = [%.4f, %.4f, %.4f] (norm=%.6f)", armName.c_str(),
                        vec_hand_to_artwork.x(), vec_hand_to_artwork.y(), vec_hand_to_artwork.z(), vec_norm);

            Eigen::Vector3d hand_x_axis = R_hand.col(0);
            double cos_angle = std::clamp(hand_x_axis.dot(vec_hand_to_artwork), -1.0, 1.0);
            Eigen::Vector3d rotation_axis = hand_x_axis.cross(vec_hand_to_artwork);
            double axis_norm = rotation_axis.norm();
            if (axis_norm <= 1e-6) {
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: rotation axis negligible, orientation likely already aligned", armName.c_str());
                // build q_target = current orientation
                Eigen::Quaterniond q_target(R_hand);
                // reachability + go_to_pose as in original flow
                yarp::os::Bottle cmd_check, res_check;
                cmd_check.addString("is_pose_reachable");
                cmd_check.addFloat64(hand_x);
                cmd_check.addFloat64(hand_y);
                cmd_check.addFloat64(hand_z);
                cmd_check.addFloat64(q_target.x());
                cmd_check.addFloat64(q_target.y());
                cmd_check.addFloat64(q_target.z());
                cmd_check.addFloat64(q_target.w());
                bool ok_check = activePort->write(cmd_check, res_check);
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: is_pose_reachable write_ok=%d, res_size=%ld", armName.c_str(), ok_check, res_check.size());
                if (!ok_check || res_check.size() == 0 || res_check.get(0).asVocab32() != yarp::os::createVocab32('o','k')) {
                    RCLCPP_WARN(m_node->get_logger(), "POSE NOT REACHABLE: %s hand cannot reach current pose", armName.c_str());
                    continue;
                }
                yarp::os::Bottle cmd_pose, res_pose;
                cmd_pose.addString("go_to_pose");
                cmd_pose.addFloat64(hand_x); cmd_pose.addFloat64(hand_y); cmd_pose.addFloat64(hand_z);
                cmd_pose.addFloat64(q_target.x()); cmd_pose.addFloat64(q_target.y()); cmd_pose.addFloat64(q_target.z()); cmd_pose.addFloat64(q_target.w());
                cmd_pose.addFloat64(15.0);
                bool ok_pose = activePort->write(cmd_pose, res_pose);
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: go_to_pose write_ok=%d, res_size=%ld", armName.c_str(), ok_pose, res_pose.size());
                if (ok_pose && res_pose.size() > 0 && res_pose.get(0).asVocab32() == yarp::os::createVocab32('o','k')) {
                    RCLCPP_INFO(m_node->get_logger(), "POSITIONING SUCCESS: %s hand aligned with artwork [%s]", armName.c_str(), request->dance_name.c_str());
                    break; // done
                } else {
                    RCLCPP_WARN(m_node->get_logger(), "POSITIONING FAILED: %s hand could not move (no-rotation case)", armName.c_str());
                    continue;
                }
            } else {
                rotation_axis.normalize();
                double angle = std::acos(cos_angle);
                Eigen::AngleAxisd rot(angle, rotation_axis);
                Eigen::Matrix3d R_target = rot * R_hand;
                Eigen::Quaterniond q_target(R_target);

                // reachability
                yarp::os::Bottle cmd_check, res_check;
                cmd_check.addString("is_pose_reachable");
                cmd_check.addFloat64(hand_x); cmd_check.addFloat64(hand_y); cmd_check.addFloat64(hand_z);
                cmd_check.addFloat64(q_target.x()); cmd_check.addFloat64(q_target.y()); cmd_check.addFloat64(q_target.z()); cmd_check.addFloat64(q_target.w());
                bool ok_check = activePort->write(cmd_check, res_check);
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: is_pose_reachable write_ok=%d, res_size=%ld", armName.c_str(), ok_check, res_check.size());
                if (!ok_check || res_check.size() == 0 || res_check.get(0).asVocab32() != yarp::os::createVocab32('o','k')) {
                    RCLCPP_WARN(m_node->get_logger(), "POSE NOT REACHABLE: %s hand cannot reach rotated pose", armName.c_str());
                    continue;
                }
                // go_to_pose
                yarp::os::Bottle cmd_pose, res_pose;
                cmd_pose.addString("go_to_pose");
                cmd_pose.addFloat64(hand_x); cmd_pose.addFloat64(hand_y); cmd_pose.addFloat64(hand_z);
                cmd_pose.addFloat64(q_target.x()); cmd_pose.addFloat64(q_target.y()); cmd_pose.addFloat64(q_target.z()); cmd_pose.addFloat64(q_target.w());
                cmd_pose.addFloat64(15.0);
                bool ok_pose = activePort->write(cmd_pose, res_pose);
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: go_to_pose write_ok=%d, res_size=%ld", armName.c_str(), ok_pose, res_pose.size());
                if (ok_pose && res_pose.size() > 0 && res_pose.get(0).asVocab32() == yarp::os::createVocab32('o','k')) {
                    RCLCPP_INFO(m_node->get_logger(), "POSITIONING SUCCESS: %s hand moved to rotated pose aligned with artwork [%s]", armName.c_str(), request->dance_name.c_str());
                    break; // success, don't try the other arm
                } else {
                    RCLCPP_WARN(m_node->get_logger(), "POSITIONING FAILED: %s hand could not move to rotated pose", armName.c_str());
                    continue;
                }
            }
        }
    }

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

