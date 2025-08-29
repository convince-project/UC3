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
#include <thread>
#include <cstdlib>                       // per std::system (avvio controller esterni)
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>

using json = nlohmann::json;

/*
 * start()
 * - Inizializza ROS2 (se necessario), il buffer TF2 e il listener.
 * - Carica le coordinate degli artwork dal file JSON.
 * - Nota: apertura/connessione delle porte YARP va fatta qui (omessa nel patch).
 */
bool ExecuteDanceComponent::start(int argc, char* argv[])
{
    // Init ROS2 if needed
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    // Create node early to use logger during startup
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");

    // Cartesian server port names expected
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";

    // Ensure YARP network available
    if (!yarp::os::Network::checkNetwork()) {
        RCLCPP_WARN(m_node->get_logger(), "YARP network not available, cannot open cartesian ports");
        return false;
    }

    // Decide INI paths: prefer member paths if set, otherwise env / default
    const std::string iniLeft = !cartesianControllerIniPathLeft.empty()
                                ? cartesianControllerIniPathLeft
                                : (std::getenv("CARTESIAN_LEFT_INI") ? std::getenv("CARTESIAN_LEFT_INI") : "/home/user1/UC3/conf/r1_cartesian_left.ini");
    const std::string iniRight = !cartesianControllerIniPathRight.empty()
                                 ? cartesianControllerIniPathRight
                                 : (std::getenv("CARTESIAN_RIGHT_INI") ? std::getenv("CARTESIAN_RIGHT_INI") : "/home/user1/UC3/conf/r1_cartesian_right.ini");

    // If remote ports missing, try to start controllers in background
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_INFO(m_node->get_logger(), "Starting r1-cartesian-control LEFT...");
        std::string cmd = "r1-cartesian-control --from " + iniLeft + " > /tmp/r1-cartesian-left.log 2>&1 &";
        int ret = std::system(cmd.c_str());
        if (ret < 0) {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to spawn left cartesian controller");
        } else {
            RCLCPP_INFO(m_node->get_logger(), "Spawned left controller (cmd rc=%d), waiting for port...", ret);
        }
    }
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_INFO(m_node->get_logger(), "Starting r1-cartesian-control RIGHT...");
        std::string cmd = "r1-cartesian-control --from " + iniRight + " > /tmp/r1-cartesian-right.log 2>&1 &";
        int ret = std::system(cmd.c_str());
        if (ret < 0) {
            RCLCPP_ERROR(m_node->get_logger(), "Failed to spawn right cartesian controller");
        } else {
            RCLCPP_INFO(m_node->get_logger(), "Spawned right controller (cmd rc=%d), waiting for port...", ret);
        }
    }

    // Wait (indefinitely) for remote cartesian ports to register with NameServer
    RCLCPP_INFO(m_node->get_logger(), "Waiting for remote cartesian ports to appear...");
    while (!yarp::os::Network::exists(cartesianPortLeft) || !yarp::os::Network::exists(cartesianPortRight)) {
        if (!yarp::os::Network::exists(cartesianPortLeft))
            RCLCPP_DEBUG(m_node->get_logger(), "Still waiting for %s ...", cartesianPortLeft.c_str());
        if (!yarp::os::Network::exists(cartesianPortRight))
            RCLCPP_DEBUG(m_node->get_logger(), "Still waiting for %s ...", cartesianPortRight.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(m_node->get_logger(), "Remote cartesian ports available.");

    // Open and connect ActionPlayer client port (robust pattern)
    yAPClientPortName = "/ExecuteDanceComponent/yarpActionsPlayerClient/rpc";
    if (yarp::os::Network::exists(yAPClientPortName)) {
        RCLCPP_WARN(m_node->get_logger(), "Port %s already exists, attempting cleanup", yAPClientPortName.c_str());
        yarp::os::Network::disconnect(yAPClientPortName, "/yarpActionsPlayer/rpc");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!m_yAPClientPort.open(yAPClientPortName)) {
        RCLCPP_ERROR(m_node->get_logger(), "Cannot open yarpActionsPlayer client port %s", yAPClientPortName.c_str());
    } else {
        yarp::os::Network::connect(yAPClientPortName, "/yarpActionsPlayer/rpc");
        RCLCPP_INFO(m_node->get_logger(), "Opened ActionPlayer client port %s", yAPClientPortName.c_str());
    }

    // Open and connect cartesian client ports (left & right)
    m_cartesianPortNameLeft  = "/ExecuteDanceComponent/cartesianClientLeft/rpc";
    m_cartesianPortNameRight = "/ExecuteDanceComponent/cartesianClientRight/rpc";

    if (yarp::os::Network::exists(m_cartesianPortNameLeft)) {
        RCLCPP_WARN(m_node->get_logger(), "Port %s already exists, attempting cleanup", m_cartesianPortNameLeft.c_str());
        yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianPortLeft);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!m_cartesianPortLeft.open(m_cartesianPortNameLeft)) {
        RCLCPP_ERROR(m_node->get_logger(), "Cannot open Left CartesianController client port %s", m_cartesianPortNameLeft.c_str());
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameLeft, cartesianPortLeft);
    RCLCPP_INFO(m_node->get_logger(), "Connected %s -> %s", m_cartesianPortNameLeft.c_str(), cartesianPortLeft.c_str());

    if (yarp::os::Network::exists(m_cartesianPortNameRight)) {
        RCLCPP_WARN(m_node->get_logger(), "Port %s already exists, attempting cleanup", m_cartesianPortNameRight.c_str());
        yarp::os::Network::disconnect(m_cartesianPortNameRight, cartesianPortRight);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    if (!m_cartesianPortRight.open(m_cartesianPortNameRight)) {
        RCLCPP_ERROR(m_node->get_logger(), "Cannot open Right CartesianController client port %s", m_cartesianPortNameRight.c_str());
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameRight, cartesianPortRight);
    RCLCPP_INFO(m_node->get_logger(), "Connected %s -> %s", m_cartesianPortNameRight.c_str(), cartesianPortRight.c_str());

    // Load artwork coordinates (if not already)
    if (m_artworkCoords.empty()) {
        m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");
    }
    RCLCPP_INFO(m_node->get_logger(), "Loaded %zu artwork entries", m_artworkCoords.size());

    // Create service to trigger executeTask
    m_executeDanceService = m_node->create_service<execute_dance_interfaces::srv::ExecuteDance>(
        "/ExecuteDanceComponent/ExecuteDance",
        [this](const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
               std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response)
        {
            this->executeTask(request);
            response->is_ok = true;
            response->error_msg = "";
        });

    RCLCPP_DEBUG(m_node->get_logger(), "ExecuteDanceComponent::start completed");
    return true;
}

/*
 * close()
 * - Chiude porte e rilascia risorse TF/ROS.
 */
bool ExecuteDanceComponent::close()
{
    if (m_cartesianPortLeft.isOpen()) m_cartesianPortLeft.close();
    if (m_cartesianPortRight.isOpen()) m_cartesianPortRight.close();
    if (m_tf_buffer) m_tf_buffer.reset();
    if (m_tf_listener) m_tf_listener.reset();
    if (rclcpp::ok()) rclcpp::shutdown();
    return true;
}

/*
 * spin()
 * - Utility minima per "spinare" il nodo ROS se necessario.
 * - Qui lasciamo un thread minimal che chiama spin_some su un nodo 'dummy' (comportamento ridotto).
 */
void ExecuteDanceComponent::spin()
{
    // Se TF inizializzati, si crea un piccolo thread per chiamare spin_some periodicamente.
    if (m_tf_buffer && m_tf_listener) {
        std::thread([](){ rclcpp::spin_some(rclcpp::Node::make_shared("dummy")); }).detach();
    }
}

/*
 * loadArtworkCoordinates(filename)
 * - Legge il JSON degli artwork e ritorna una mappa: nome -> {x,y,z}.
 * - Formato JSON atteso:
 *   { "artworks": { "name1": {"x":..., "y":..., "z":...}, "name2": {...} } }
 */
std::map<std::string, std::vector<double>> ExecuteDanceComponent::loadArtworkCoordinates(const std::string& filename)
{
    std::map<std::string, std::vector<double>> out;
    std::ifstream ifs(filename);
    if (!ifs.good()) return out;
    json j; ifs >> j;
    if (!j.contains("artworks")) return out;
    for (auto it = j["artworks"].begin(); it != j["artworks"].end(); ++it) {
        const std::string name = it.key();
        double x = it.value().value("x", 0.0);
        double y = it.value().value("y", 0.0);
        double z = it.value().value("z", 0.0);
        out[name] = {x,y,z};
    }
    return out;
}

/*
 * preScanArticulatedArms(artwork_pos)
 * - Tenta di leggere le trasformate delle spalle (right/left) rispetto a base_link.
 * - Calcola la distanza artwork <-> spalla e ritorna una lista di (ARMNAME, distanza).
 * - Serve per ordinare i bracci per prossimità all'artwork.
 * - Non dipende dalla posa della mano (approccio shoulder-only).
 */
bool ExecuteDanceComponent::preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                                   std::vector<std::pair<std::string,double>>& armDistances,
                                                   std::map<std::string, std::vector<double>>& /*cachedPoseValues*/)
{
    armDistances.clear();

    // Prova a leggere TF della spalla destra; se disponibile calcola distanza
    try {
        auto tfst = m_tf_buffer->lookupTransform("base_link", m_r_shoulder_frame_id, tf2::TimePointZero);
        Eigen::Vector3d shoulder(tfst.transform.translation.x,
                                 tfst.transform.translation.y,
                                 tfst.transform.translation.z);
        double d = (artwork_pos - shoulder).norm();
        armDistances.emplace_back("RIGHT", d);
    } catch (...) {
        // Se TF mancante, semplicemente skip
    }

    // Stessa cosa per la spalla sinistra
    try {
        auto tfst = m_tf_buffer->lookupTransform("base_link", m_l_shoulder_frame_id, tf2::TimePointZero);
        Eigen::Vector3d shoulder(tfst.transform.translation.x,
                                 tfst.transform.translation.y,
                                 tfst.transform.translation.z);
        double d = (artwork_pos - shoulder).norm();
        armDistances.emplace_back("LEFT", d);
    } catch (...) {
        // skip
    }

    return !armDistances.empty();
}

/*
 * executeTask(request)
 * - Flusso principale: dato il nome dell'artwork:
 *   1) recupera coordinate artwork dal file
 *   2) esegue preScan per trovare quali spalle sono disponibili
 *   3) per ogni braccio (dal più vicino) calcola:
 *      - posizione della spalla via TF
 *      - direzione shoulder->artwork
 *      - orientamento allineato (getAlignedRotation)
 *      - punto "reachable" limitato dal raggio di reach (reachablePointSphere)
 *      - verifica raggiungibilità (isPoseReachable) e invio go_to_pose al controller cartesiano
 *      - se non raggiungibile prova un probing binario lungo la retta shoulder->reachable (probeBinarySearch)
 *   4) termina quando un braccio ha eseguito con successo il go_to_pose
 *
 * Note:
 * - Tutto il controllo di reachability e il movimento viene mandato al controller cartesiano via YARP RPC
 *   (porte m_cartesianPortLeft / m_cartesianPortRight).
 */
void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    RCLCPP_INFO(m_node->get_logger(), "executeTask() called for '%s'", request->dance_name.c_str());

    auto it = m_artworkCoords.find(request->dance_name);
    if (it == m_artworkCoords.end()) {
        RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "Artwork '%s' not found", request->dance_name.c_str());
        return;
    }
    Eigen::Vector3d artwork_pos(it->second[0], it->second[1], it->second[2]);
    RCLCPP_INFO(m_node->get_logger(), "Artwork position (base_link): [%.3f, %.3f, %.3f]", artwork_pos.x(), artwork_pos.y(), artwork_pos.z());

    std::vector<std::pair<std::string,double>> armDistances;
    std::map<std::string,std::vector<double>> dummy_cache;
    if (!preScanArticulatedArms(artwork_pos, armDistances, dummy_cache)) {
        RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "No shoulder TFs available, cannot perform shoulder-based pointing");
        return;
    }

    // Ordina i bracci per distanza (più vicino primo)
    std::sort(armDistances.begin(), armDistances.end(), [](const auto&a,const auto&b){return a.second<b.second;});

    // debug: elenco bracci disponibili e distanze
    for (const auto &p : armDistances) {
        RCLCPP_INFO(m_node->get_logger(), "Available arm: %s distance=%.3f", p.first.c_str(), p.second);
    }

    for (const auto &entry : armDistances) {
        const std::string armName = entry.first;
        RCLCPP_INFO(m_node->get_logger(), "Processing arm %s", armName.c_str());
        // seleziona la porta YARP corretta per il braccio
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;

        // Legge la trasformata della spalla (necessaria per il metodo shoulder-only)
        const std::string shoulder_frame = (armName == "LEFT") ? m_l_shoulder_frame_id : m_r_shoulder_frame_id;
        Eigen::Vector3d shoulder_pos;
        try {
            auto tfst = m_tf_buffer->lookupTransform("base_link", shoulder_frame, tf2::TimePointZero);
            shoulder_pos = {tfst.transform.translation.x, tfst.transform.translation.y, tfst.transform.translation.z};
            RCLCPP_INFO(m_node->get_logger(), "Arm %s shoulder_pos = [%.3f, %.3f, %.3f]", armName.c_str(), shoulder_pos.x(), shoulder_pos.y(), shoulder_pos.z());
        } catch (const std::exception &e) {
            RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "TF lookup failed for %s: %s", shoulder_frame.c_str(), e.what());
            continue; // passa al prossimo braccio
        }

        // Calcola la direzione shoulder -> artwork e normalizza
        Eigen::Vector3d dir = artwork_pos - shoulder_pos;
        double dir_norm = dir.norm();
        if (dir_norm <= 1e-9) {
            RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "degenerate shoulder->artwork vector for arm %s", armName.c_str());
            continue;
        }
        dir.normalize();
        RCLCPP_INFO(m_node->get_logger(), "Arm %s dir (shoulder->artwork) = [%.3f, %.3f, %.3f]", armName.c_str(), dir.x(), dir.y(), dir.z());

        // Calcola l'orientamento dell'end-effector allineato con la direzione
        Eigen::Matrix3d R_aligned = getAlignedRotation(dir);
        auto [qx,qy,qz,qw] = rot2quats(R_aligned);
        Eigen::Quaterniond q_target(qw,qx,qy,qz);
        RCLCPP_INFO(m_node->get_logger(), "Arm %s q_target = [x=%.6f y=%.6f z=%.6f w=%.6f]", armName.c_str(), q_target.x(), q_target.y(), q_target.z(), q_target.w());

        // Punto massimo 'raggiungibile' lungo la direzione, limitato da m_reach_radius
        Eigen::Vector3d reachable = reachablePointSphere(shoulder_pos, artwork_pos, m_reach_radius);
        RCLCPP_INFO(m_node->get_logger(), "Arm %s reachable point = [%.3f, %.3f, %.3f]", armName.c_str(), reachable.x(), reachable.y(), reachable.z());

        // Primo tentativo: se 'reachable' è raggiungibile manda direttamente il comando go_to_pose
        if (isPoseReachable(activePort, reachable, q_target)) {
            yarp::os::Bottle cmd,res;
            cmd.addString("go_to_pose");
            cmd.addFloat64(reachable.x()); cmd.addFloat64(reachable.y()); cmd.addFloat64(reachable.z());
            cmd.addFloat64(q_target.x()); cmd.addFloat64(q_target.y()); cmd.addFloat64(q_target.z()); cmd.addFloat64(q_target.w());
            cmd.addFloat64(15.0); // durata traiettoria (s)
            bool ok = activePort->write(cmd,res);
            if (ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
                RCLCPP_INFO(rclcpp::get_logger("ExecuteDanceComponent"), "ARM %s: positioned at reachable point", armName.c_str());
                break; // successo, esci
            } else {
                RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "ARM %s: go_to_pose failed for reachable point", armName.c_str());
                continue;
            }
        }

        // Se non raggiungibile, esegui probing binario lungo la retta shoulder->reachable per trovare
        // il punto più lontano raggiungibile (probeBinarySearch usa isPoseReachable internamente).
        double vec_norm_sh = (reachable - shoulder_pos).norm();
        if (vec_norm_sh <= 1e-9) {
            RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "reachable coincident with shoulder for arm %s", armName.c_str());
            continue;
        }
        Eigen::Vector3d dir_sh = (reachable - shoulder_pos).normalized();
        Eigen::Vector3d best_candidate;
        if (!probeBinarySearch(activePort, shoulder_pos, reachable, dir_sh, vec_norm_sh, q_target, best_candidate)) {
            RCLCPP_WARN(m_node->get_logger(), "probing failed for arm %s", armName.c_str());
            continue;
        }
        RCLCPP_INFO(m_node->get_logger(), "Arm %s best_candidate from probing = [%.3f, %.3f, %.3f]", armName.c_str(), best_candidate.x(), best_candidate.y(), best_candidate.z());

        // Invio comando finale per il candidato trovato dal probing
        yarp::os::Bottle cmd_final, res_final;
        cmd_final.addString("go_to_pose");
        cmd_final.addFloat64(best_candidate.x()); cmd_final.addFloat64(best_candidate.y()); cmd_final.addFloat64(best_candidate.z());
        cmd_final.addFloat64(q_target.x()); cmd_final.addFloat64(q_target.y()); cmd_final.addFloat64(q_target.z()); cmd_final.addFloat64(q_target.w());
        cmd_final.addFloat64(15.0);
        bool ok_final = activePort->write(cmd_final, res_final);
        if (ok_final && res_final.size()>0 && res_final.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
            RCLCPP_INFO(rclcpp::get_logger("ExecuteDanceComponent"), "ARM %s: positioned (final candidate)", armName.c_str());
            break;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("ExecuteDanceComponent"), "ARM %s: go_to_pose failed for final candidate", armName.c_str());
            continue;
        }
    } // fine ciclo bracci
}

/*
 * isPoseReachable(activePort, candidate, q_target)
 * - Chiede al controller cartesiano via RPC "is_pose_reachable".
 * - Ritorna true se la risposta è 'ok' (vocab 'o','k').
 */
bool ExecuteDanceComponent::isPoseReachable(yarp::os::Port* activePort,
                                            const Eigen::Vector3d& candidate,
                                            const Eigen::Quaterniond& q_target)
{
    if (!activePort) return false;
    yarp::os::Bottle cmd, res;
    cmd.addString("is_pose_reachable");
    cmd.addFloat64(candidate.x()); cmd.addFloat64(candidate.y()); cmd.addFloat64(candidate.z());
    cmd.addFloat64(q_target.x()); cmd.addFloat64(q_target.y()); cmd.addFloat64(q_target.z()); cmd.addFloat64(q_target.w());
    bool ok = activePort->write(cmd, res);
    if (!ok || res.size()==0) return false;
    return res.get(0).asVocab32() == yarp::os::createVocab32('o','k');
}

/*
 * probeBinarySearch(...)
 * - Esegue una ricerca binaria sul parametro t della retta: candidate = start_pos + dir * t.
 * - Cerca il valore massimo di t (entro max_t = dir_norm - safety_margin) per cui isPoseReachable è true.
 * - Ritorna il miglior candidato in out_best_candidate.
 */
bool ExecuteDanceComponent::probeBinarySearch(yarp::os::Port* activePort,
                                              const Eigen::Vector3d& start_pos,
                                              const Eigen::Vector3d& /*end_pos*/,
                                              const Eigen::Vector3d& dir,
                                              double dir_norm,
                                              const Eigen::Quaterniond& q_target,
                                              Eigen::Vector3d& out_best_candidate)
{
    const double safety_margin = 0.05; // margine per sicurezza (non provare esattamente fino all'artwork)
    const double pos_tol = 1e-3;       // tolleranza posizione per terminare la ricerca
    const int max_iters = 20;
 
    double max_t = std::max(0.0, dir_norm - safety_margin);
    Eigen::Vector3d candidate_max = start_pos + dir * max_t;
    RCLCPP_DEBUG(rclcpp::get_logger("ExecuteDanceComponent"), "probeBinarySearch: max_t=%.3f candidate_max=[%.3f,%.3f,%.3f]", max_t, candidate_max.x(), candidate_max.y(), candidate_max.z());
    if (isPoseReachable(activePort, candidate_max, q_target)) {
        out_best_candidate = candidate_max;
        RCLCPP_DEBUG(rclcpp::get_logger("ExecuteDanceComponent"), "probeBinarySearch: candidate_max reachable");
        return true;
    }
 
    double lo = 0.0;
    double hi = max_t;
    double best_t = 0.0;
    for (int it = 0; it < max_iters && (hi - lo) > pos_tol; ++it) {
        double mid = 0.5 * (lo + hi);
        Eigen::Vector3d candidate = start_pos + dir * mid;
        RCLCPP_DEBUG(rclcpp::get_logger("ExecuteDanceComponent"), "probeBinarySearch it=%d lo=%.4f hi=%.4f mid=%.4f candidate=[%.3f,%.3f,%.3f]",
                    it, lo, hi, mid, candidate.x(), candidate.y(), candidate.z());
        if (isPoseReachable(activePort, candidate, q_target)) {
            best_t = mid;
            lo = mid;
        } else {
            hi = mid;
        }
    }
    out_best_candidate = start_pos + dir * best_t;
    RCLCPP_DEBUG(rclcpp::get_logger("ExecuteDanceComponent"), "probeBinarySearch: best_t=%.4f best_candidate=[%.3f,%.3f,%.3f]", best_t, out_best_candidate.x(), out_best_candidate.y(), out_best_candidate.z());
    return true;
}

/*
 * reachablePointSphere(shoulder, target, reach_radius)
 * - Calcola il punto lungo la retta shoulder->target alla distanza r = min(reach_radius, dist(shoulder,target))
 * - Se la distanza è molto piccola, ritorna la posizione della spalla.
 */
Eigen::Vector3d ExecuteDanceComponent::reachablePointSphere(const Eigen::Vector3d& shoulder,
                                                            const Eigen::Vector3d& target,
                                                            double reach_radius) const
{
    Eigen::Vector3d d = target - shoulder;
    double dist = d.norm();
    if (dist <= 1e-9) return shoulder;
    double r = std::min(reach_radius, dist);
    return shoulder + d.normalized() * r;
}

/*
 * getAlignedRotation(v)
 * - Costruisce una matrice di rotazione 3x3 dove:
 *   colonna 2 (z) = -v (l'end-effector guarda verso il target)
 *   colonna 1 (y) = vettore ortogonale "up" ottenuto con Gram-Schmidt usando world_y
 *   colonna 0 (x) = cross(ee_y, ee_z)
 * - Gestisce il caso in cui v è quasi allineato con world_y usando world_x come riferimento alternativo.
 */
Eigen::Matrix3d ExecuteDanceComponent::getAlignedRotation(const Eigen::Vector3d& v) const
{
    Eigen::Vector3d ee_z = -v.normalized(); // z dell'EE punta verso il target (verso -v)
    const Eigen::Vector3d world_y(0.0,1.0,0.0);
    const Eigen::Vector3d world_x(1.0,0.0,0.0);
    const double eps = 1e-6;

    Eigen::Vector3d ee_x, ee_y;
    // Se ee_z è allineato con world_y usiamo world_x per evitare degenerazione
    if (std::abs(world_y.dot(ee_z)) > (1.0 - eps)) {
        ee_y = ee_z.cross(world_x).normalized();
        ee_x = ee_y.cross(ee_z).normalized();
    } else {
        ee_y = (world_y - (world_y.dot(ee_z))*ee_z).normalized();
        ee_x = ee_y.cross(ee_z).normalized();
    }

    Eigen::Matrix3d R;
    R.col(0) = ee_x;
    R.col(1) = ee_y;
    R.col(2) = ee_z;
    return R;
}

/*
 * rot2quats(R)
 * - Converte la matrice di rotazione (Eigen::Matrix3d) in quaternion Eigen::Quaterniond.
 * - Ritorna la tupla (x,y,z,w) — il caller costruisce Eigen::Quaterniond(w,x,y,z).
 */
std::tuple<double,double,double,double> ExecuteDanceComponent::rot2quats(const Eigen::Matrix3d& R) const
{
    Eigen::Quaterniond q(R);
    return {q.x(), q.y(), q.z(), q.w()};
}

