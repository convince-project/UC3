/******************************************************************************
 *                                                                            *
 * UC3 — ExecuteDanceComponent (implementation)                                *
 *                                                                            *
 * Variante: camera-style (punto candidato su retta spalla→target)            *
 * Nota: fix yaw con tf2::Matrix3x3(...).getRPY()                              *
 *                                                                            *
 ******************************************************************************/
#include "ExecuteDanceComponent.h"

// ==== C++ std ====
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>

// ==== JSON (nlohmann) per caricare le coordinate delle opere ====
#include <nlohmann/json.hpp>
using ordered_json = nlohmann::ordered_json;

// ==== TF2 LinearMath per la conversione Quaternion -> RPY (fix getYaw) ====
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// =============================================================================
// start() — avvio di ROS2, TF2, porte YARP e servizio ROS2
// =============================================================================
bool ExecuteDanceComponent::start(int argc, char* argv[])
{
    // 1) Inizializza ROS2 se necessario
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }

    // 2) Porte RPC dei controller cartesiani che ci aspettiamo di usare
    const std::string cartesianPortLeft  = "/r1-cartesian-control/left_arm/rpc:i";
    const std::string cartesianPortRight = "/r1-cartesian-control/right_arm/rpc:i";

    // 3) Avvia i controller se non sono già attivi
    if (!yarp::os::Network::exists(cartesianPortLeft)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avvio r1-cartesian-control LEFT...");
        std::string cmd = std::string("r1-cartesian-control --from ") + cartesianControllerIniPathLeft + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nell'avvio di r1-cartesian-control LEFT");
            return false;
        }
    }
    if (!yarp::os::Network::exists(cartesianPortRight)) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avvio r1-cartesian-control RIGHT...");
        std::string cmd = std::string("r1-cartesian-control --from ") + cartesianControllerIniPathRight + " > /dev/null 2>&1 &";
        if (std::system(cmd.c_str()) == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Errore nell'avvio di r1-cartesian-control RIGHT");
            return false;
        }
    }

    // 4) Attendi che i server RPC compaiano (no timeout: comodo in dev)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attesa della porta %s...", cartesianPortLeft.c_str());
    while (!yarp::os::Network::exists(cartesianPortLeft)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attesa della porta %s...", cartesianPortRight.c_str());
    while (!yarp::os::Network::exists(cartesianPortRight)) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 5) Crea il nodo ROS2 e il TF2 persistente (buffer+listener)
    m_node = rclcpp::Node::make_shared("ExecuteDanceComponentNode");
    m_tfBuffer   = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // 6) Carica le coordinate delle opere (in frame "map")
    m_artworkCoords = loadArtworkCoordinates("/home/user1/UC3/conf/artwork_coords.json");

    // 7) Apri e connetti le porte RPC locali verso i due controller cartesiani
    m_cartesianPortNameLeft  = "/ExecuteDanceComponent/cartesianClientLeft/rpc";
    m_cartesianPortNameRight = "/ExecuteDanceComponent/cartesianClientRight/rpc";

    if (yarp::os::Network::exists(m_cartesianPortNameLeft)) {
        yarp::os::Network::disconnect(m_cartesianPortNameLeft, cartesianPortLeft);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!m_cartesianPortLeft.open(m_cartesianPortNameLeft)) {
        yError() << "Cannot open Left CartesianController client port";
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameLeft, cartesianPortLeft);

    if (yarp::os::Network::exists(m_cartesianPortNameRight)) {
        yarp::os::Network::disconnect(m_cartesianPortNameRight, cartesianPortRight);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!m_cartesianPortRight.open(m_cartesianPortNameRight)) {
        yError() << "Cannot open Right CartesianController client port";
        return false;
    }
    yarp::os::Network::connect(m_cartesianPortNameRight, cartesianPortRight);

    // 8) Esponi il servizio ROS2 per avviare il pointing verso un'opera (per nome)
    m_executeDanceService = m_node->create_service<execute_dance_interfaces::srv::ExecuteDance>(
        "/ExecuteDanceComponent/ExecuteDance",
        [this](const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request,
               std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Response> response)
        {
            this->executeTask(request); // esegue in-line; per threading, spostare su un worker
            response->is_ok = true;
            response->error_msg = "";
        }
    );

    return true;
}

// =============================================================================
// close() — chiude porte e spegne ROS2
// =============================================================================
bool ExecuteDanceComponent::close()
{
    if (m_cartesianClient.isValid()) {
        m_cartesianClient.close();
    }
    if (m_cartesianPortLeft.isOpen())  m_cartesianPortLeft.close();
    if (m_cartesianPortRight.isOpen()) m_cartesianPortRight.close();
    rclcpp::shutdown();
    return true;
}

void ExecuteDanceComponent::spin()
{
    rclcpp::spin(m_node);
}

// =============================================================================
// executeTask() — cuore della logica di pointing (camera-style)
// =============================================================================
void ExecuteDanceComponent::executeTask(const std::shared_ptr<execute_dance_interfaces::srv::ExecuteDance::Request> request)
{
    RCLCPP_INFO(m_node->get_logger(), "EXECUTE TASK: '%s'", request->dance_name.c_str());

    // 1) Recupera le coordinate dell'opera (in frame map)
    auto it = m_artworkCoords.find(request->dance_name);
    if (it == m_artworkCoords.end() || it->second.size() < 3) {
        RCLCPP_WARN(m_node->get_logger(), "No or invalid ARTWORK '%s' found", request->dance_name.c_str());
        return;
    }
    const auto& coords = it->second;

    // 2) Trasforma il punto da map → base_link (se TF disponibile); fallback: usa map come base
    Eigen::Vector3d p_base{coords[0], coords[1], coords[2]};
    geometry_msgs::msg::Point mapPt; mapPt.x = coords[0]; mapPt.y = coords[1]; mapPt.z = coords[2];
    geometry_msgs::msg::Point basePt;
    if (transformPointMapToRobot(mapPt, basePt, m_baseFrame, 1.0)) {
        p_base = Eigen::Vector3d(basePt.x, basePt.y, basePt.z);
    } else {
        RCLCPP_WARN(m_node->get_logger(), "TF map->%s failed; assuming coords already in base", m_baseFrame.c_str());
    }

    // 3) Pre-scan: interroga i due controller per la posa mano e stima distanza dal target
    std::vector<std::pair<std::string,double>> armDistances;                 // (arm, distanza)
    std::map<std::string, std::vector<double>> cachedPoseValues;             // (arm -> flat pose)
    if (!preScanArticulatedArms(p_base, armDistances, cachedPoseValues)) {
        RCLCPP_ERROR(m_node->get_logger(), "No arms available for pointing");
        return;
    }

    // 4) Scegli braccio : segno della Y nel frame "torso"
    std::string preferredArm;
    if (!chooseArmByTorsoY(p_base, preferredArm)) preferredArm = "LEFT";     // fallback
    std::stable_sort(armDistances.begin(), armDistances.end(), [&](auto&a, auto&b){
        if (a.first==preferredArm && b.first!=preferredArm) return true;     // preferito prima
        if (a.first!=preferredArm && b.first==preferredArm) return false;
        return a.second < b.second;                                          // poi per distanza
    });

    // 5) Prova i bracci nell'ordine stabilito 
    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;                           // "LEFT" o "RIGHT"
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                           : "/r1-cartesian-control/right_arm/rpc:i";
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready, skipping", armName.c_str());
            continue;
        }

        // 5.a) Orientazione: direzione spalla→target in BASE, allineo l'asse EEF scelto
        Eigen::Vector3d shoulder_base;
        if (!getShoulderPosInBase(armName, shoulder_base)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: missing shoulder TF", armName.c_str());
            continue;
        }
        Eigen::Vector3d dir_base = p_base - shoulder_base;                    // direzione di pointing
        if (dir_base.norm() < 1e-6) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: degenerate target near shoulder", armName.c_str());
            continue;
        }
        Eigen::Quaterniond q_target = quatAlignAxisToDir(dir_base, m_toolAxis, Eigen::Vector3d::UnitY());

        // 5.b) Punto candidato "camera-style": sulla retta spalla→target, a raggio limitato
        Eigen::Vector3d candidate = sphereReachPoint(shoulder_base, p_base);

        // 5.c) Se il candidato non è raggiungibile, prova direttamente l'altro braccio
        if (!isPoseReachable(activePort, candidate, q_target)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: candidate not reachable, trying next arm", armName.c_str());
            continue;
        }

        // 5.d) Invia il movimento finale al controller cartesiano
        yarp::os::Bottle cmd_pose_final, res_pose_final;
        cmd_pose_final.addString("go_to_pose");
        cmd_pose_final.addFloat64(candidate.x());
        cmd_pose_final.addFloat64(candidate.y());
        cmd_pose_final.addFloat64(candidate.z());
        cmd_pose_final.addFloat64(q_target.x());
        cmd_pose_final.addFloat64(q_target.y());
        cmd_pose_final.addFloat64(q_target.z());
        cmd_pose_final.addFloat64(q_target.w());
        cmd_pose_final.addFloat64(15.0);                                        // durata traiettoria (tarabile)

        bool ok = activePort->write(cmd_pose_final, res_pose_final);
        if (ok && res_pose_final.size()>0 && res_pose_final.get(0).asVocab32()==yarp::os::createVocab32('o','k')) {
            RCLCPP_INFO(m_node->get_logger(), "SUCCESS: %s aligned toward '%s'", armName.c_str(), request->dance_name.c_str());
            break; // missione compiuta
        } else {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed, trying next arm", armName.c_str());
            continue;
        }
    }
}

// =============================================================================
// amclPoseCallback() — esempio di aggiornamento stato (usa getRPY, non getYaw)
// =============================================================================
void ExecuteDanceComponent::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    m_currentX = p.x;
    m_currentY = p.y;

    // Quaternion ROS → tf2::Quaternion
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);

    // Ottieni roll,pitch,yaw in modo portabile (fix per getYaw non disponibile)
    double roll=0.0, pitch=0.0, yaw=0.0;
    tf2::Matrix3x3(qq).getRPY(roll, pitch, yaw);
    m_currentYaw = yaw;
}

// =============================================================================
// Reachability helpers & wrappers
// =============================================================================
bool ExecuteDanceComponent::checkPoseReachabilityForArm(double x, double y, double z, const std::string& armName)
{
    yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight; // seleziona la porta in base al braccio
    yarp::os::Bottle cmd, res;                                                                        // contenitori YARP per comando/risposta
    cmd.addString("is_pose_reachable");                                                               // comando RPC al controller cartesiano
    cmd.addFloat64(x); cmd.addFloat64(y); cmd.addFloat64(z);                                          // posizione da verificare
    // orientamento "neutro" (q = [0,0,0,1]) per un pre-check grossolano
    cmd.addFloat64(0.0); cmd.addFloat64(0.0); cmd.addFloat64(0.0); cmd.addFloat64(1.0);
    bool ok = activePort->write(cmd, res);                                                            // invia il comando e legge la risposta
    return ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k');            // true se RPC ok e primo token == 'ok'
}

bool ExecuteDanceComponent::checkPoseReachability(double x, double y, double z)
{
    return checkPoseReachabilityForArm(x,y,z,"LEFT"); // convenienza: controlla sul braccio sinistro per default
}

// =============================================================================
// loadArtworkCoordinates() — carica { "artworks": { name: {x,y,z} } }
// =============================================================================
std::map<std::string, std::vector<double>> ExecuteDanceComponent::loadArtworkCoordinates(const std::string& filename)
{
    std::map<std::string, std::vector<double>> artworkMap;                                           // mappa nome→[x,y,z] da riempire
    try {
        std::ifstream file(filename);                                                                 // apre il file JSON
        if (!file.is_open()) {                                                                        // verifica apertura
            RCLCPP_ERROR(m_node->get_logger(), "Unable to open artwork file '%s'", filename.c_str()); // log errore
            return artworkMap;                                                                        // ritorna vuoto
        }
        auto config = ordered_json::parse(file);                                                      // parse JSON (usa nlohmann::ordered_json)
        for (auto& [name, data] : config.at("artworks").items()) {                                    // itera sulle voci in "artworks"
            double x = data.at("x").get<double>();                                                    // legge x
            double y = data.at("y").get<double>();                                                    // legge y
            double z = data.at("z").get<double>();                                                    // legge z
            artworkMap.emplace(name, std::vector<double>{x,y,z});                                     // inserisce nella mappa
            RCLCPP_INFO(m_node->get_logger(), "Loaded artwork '%s' [%.2f %.2f %.2f]", name.c_str(), x,y,z); // log informativo
        }
    } catch (const std::exception& ex) {                                                              // cattura parsing/lookup error
        RCLCPP_ERROR(m_node->get_logger(), "Error parsing '%s': %s", filename.c_str(), ex.what());    // log errore
    }
    return artworkMap;                                                                                // ritorna la mappa (anche se parziale)
}

// =============================================================================
// transformPointMapToRobot() — helper legacy che usa un listener locale
// =============================================================================
bool ExecuteDanceComponent::transformPointMapToRobot(const geometry_msgs::msg::Point& map_point,
                                                     geometry_msgs::msg::Point& out_robot_point,
                                                     const std::string& robot_frame,
                                                     double timeout_sec)
{
    if (!m_tfBuffer) {
        RCLCPP_WARN(m_node->get_logger(), "TF buffer not initialized");
        return false;
    }

    const std::string map_frame = m_mapFrame; // tipicamente "map"
    const auto timeout = tf2::durationFromSec(timeout_sec);

    // aspetta che la TF sia disponibile
    if (!m_tfBuffer->canTransform(robot_frame, map_frame, tf2::TimePointZero, timeout)) {
        RCLCPP_WARN(m_node->get_logger(), "TF: transform %s <- %s not available", robot_frame.c_str(), map_frame.c_str());
        return false;
    }

    try {
        // lookup con il buffer persistente
        auto tf_stamped = m_tfBuffer->lookupTransform(robot_frame, map_frame, tf2::TimePointZero, timeout);

        geometry_msgs::msg::PointStamped p_in, p_out;
        p_in.header = tf_stamped.header;     // timestamp e frame di 'map'
        p_in.header.frame_id = map_frame;
        p_in.point = map_point;

        tf2::doTransform(p_in, p_out, tf_stamped);
        out_robot_point = p_out.point;
        return true;

    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(m_node->get_logger(), "TF exception: %s", ex.what());
        return false;
    }
}


// =============================================================================
// preScanArticulatedArms() — interroga get_pose e stima distanze dal target
// =============================================================================
bool ExecuteDanceComponent::preScanArticulatedArms(const Eigen::Vector3d& artwork_pos,
                                                   std::vector<std::pair<std::string,double>>& armDistances,
                                                   std::map<std::string, std::vector<double>>& cachedPoseValues)
{
    armDistances.clear();                                                                              // pulisce output distanze
    cachedPoseValues.clear();                                                                          // pulisce cache pose

    for (const std::string armId : {"LEFT", "RIGHT"}) {                                                // itera su entrambi i bracci
        yarp::os::Port* clientPort = (armId == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight; // seleziona porta
        const std::string serverPort = (armId == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"      // server remoto
                                                           : "/r1-cartesian-control/right_arm/rpc:i";
        if (!clientPort->isOpen() || !yarp::os::Network::isConnected(clientPort->getName(), serverPort)) { // verifica connessione
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not ready", armId.c_str());            // warning e salta
            continue;
        }

        // Chiedi la posa (matrice 4x4 appiattita) al controller
        yarp::os::Bottle cmd_get, res_get; 
        cmd_get.addString("get_pose");                                                                  // prepara comando get_pose
        if (!clientPort->write(cmd_get, res_get)) {                                                     // invia RPC
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", armId.c_str());       // warning
            continue;
        }

        // Unroll della Bottle in un vettore double flat (16 o 18 elementi)
        std::vector<double> flat_pose;                                                                  // vettore double appiattito
        for (size_t i = 0; i < res_get.size(); ++i) {                                                   // srotola la bottle
            if (res_get.get(i).isList()) {                                                              // caso: elemento è una lista
                yarp::os::Bottle* sub = res_get.get(i).asList();                                        // ottiene lista
                for (size_t j = 0; j < sub->size(); ++j) flat_pose.push_back(sub->get(j).asFloat64());  // aggiunge gli elementi
            } else {
                flat_pose.push_back(res_get.get(i).asFloat64());                                        // elemento singolo
            }
        }
        if (flat_pose.size() != 18 && flat_pose.size() != 16) {                                         // valida dimensione (16/18)
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: unexpected pose size=%zu for %s", flat_pose.size(), armId.c_str());
            continue;                                                                                   // salta se non attesa
        }

        // Estrai la traslazione della mano (elementi 3,7,11 con eventuale offset 2)
        const size_t pose_offset = (flat_pose.size()==18)?2u:0u;                                        // alcune risposte hanno 2 header
        const double hand_tx = flat_pose[pose_offset + 3];                                              // estrai x della traslazione
        const double hand_ty = flat_pose[pose_offset + 7];                                              // estrai y
        const double hand_tz = flat_pose[pose_offset + 11];                                             // estrai z

        // Distanza mano→target (solo per ordinare preferenze)
        const double dist = (Eigen::Vector3d(artwork_pos) - Eigen::Vector3d(hand_tx, hand_ty, hand_tz)).norm(); // distanza mano→target
        armDistances.emplace_back(armId, dist);                                                         // accumula per ordinamento
        cachedPoseValues.emplace(armId, std::move(flat_pose));                                          // salva la flat pose in cache
    }
    return !armDistances.empty();                                                                       // true se almeno un braccio letto
}

// =============================================================================
// isPoseReachable() — wrapper RPC
// =============================================================================
bool ExecuteDanceComponent::isPoseReachable(yarp::os::Port* activePort,
                                            const Eigen::Vector3d& candidate,
                                            const Eigen::Quaterniond& q_target)
{
    yarp::os::Bottle cmd, res;                                                                          // comando/risposta
    cmd.addString("is_pose_reachable");                                                                 // RPC di reachability
    cmd.addFloat64(candidate.x());                                                                      // pos x
    cmd.addFloat64(candidate.y());                                                                      // pos y
    cmd.addFloat64(candidate.z());                                                                      // pos z
    cmd.addFloat64(q_target.x());                                                                       // quat x
    cmd.addFloat64(q_target.y());                                                                       // quat y
    cmd.addFloat64(q_target.z());                                                                       // quat z
    cmd.addFloat64(q_target.w());                                                                       // quat w
    bool ok = activePort->write(cmd, res);                                                              // invia e attende risposta
    return ok && res.size()>0 && res.get(0).asVocab32()==yarp::os::createVocab32('o','k');              // true se 'ok'
}

// =============================================================================
// getTFMatrix(), transformPoint(), chooseArmByTorsoY(), getShoulderPosInBase(),
// quatAlignAxisToDir(), sphereReachPoint() — helpers geometrici/TF
// =============================================================================
bool ExecuteDanceComponent::getTFMatrix(const std::string& target,
                                        const std::string& source,
                                        Eigen::Matrix4d& T) const
{
    if (!m_tfBuffer) return false;                                                                      // buffer TF inizializzato?
    try {
        auto tf = m_tfBuffer->lookupTransform(target, source, rclcpp::Time(0), std::chrono::seconds(1)); // recupera T_target_source
        const auto& tr = tf.transform.translation;                                                      // traduzione
        const auto& q  = tf.transform.rotation;                                                         // quaternion
        Eigen::Quaterniond Q(q.w, q.x, q.y, q.z);                                                       // costruisce quaternion Eigen (w,x,y,z)
        Eigen::Matrix3d R = Q.toRotationMatrix();                                                       // converte in matrice 3x3
        T.setIdentity();                                                                                // omogenea 4x4 identità
        T.topLeftCorner<3,3>() = R;                                                                     // inserisce rotazione
        T(0,3)=tr.x; T(1,3)=tr.y; T(2,3)=tr.z;                                                          // inserisce traslazione
        return true;                                                                                    // successo
    } catch (const tf2::TransformException& e) {                                                        // errori TF
        RCLCPP_WARN(m_node->get_logger(), "TF lookup failed %s<- %s : %s", target.c_str(), source.c_str(), e.what());
        return false;                                                                                   // fallimento
    }
}

Eigen::Vector3d ExecuteDanceComponent::transformPoint(const Eigen::Matrix4d& T,
                                                      const Eigen::Vector3d& p)
{
    Eigen::Vector4d ph; ph << p, 1.0;                                                                   // omogeneizza p -> [p;1]
    ph = T*ph;                                                                                          // applica trasformazione
    return ph.head<3>();                                                                                // de-omogeneizza e ritorna 3D
}

bool ExecuteDanceComponent::chooseArmByTorsoY(const Eigen::Vector3d& p_base,
                                              std::string& outArm) const
{
    Eigen::Matrix4d T_torso_base;                                                                       // T_torso_base (torso <- base)
    if (!getTFMatrix(m_torsoFrame, m_baseFrame, T_torso_base)) return false;                            // se TF mancante → false
    Eigen::Vector3d p_torso = transformPoint(T_torso_base, p_base);                                     // esprime il target nel frame torso
    outArm = (p_torso.y() > 0.0) ? "LEFT" : "RIGHT";                                                    // y>0 → sinistro, altrimenti destro
    return true;                                                                                        // scelta riuscita
}

bool ExecuteDanceComponent::getShoulderPosInBase(const std::string& arm,
                                                 Eigen::Vector3d& out_pos) const
{
    const std::string& shoulder = (arm=="LEFT") ? m_lShoulderFrame : m_rShoulderFrame;                 // frame spalla in base al braccio
    Eigen::Matrix4d T_base_sh;                                                                          // T_base_sh (base <- shoulder)
    if (!getTFMatrix(m_baseFrame, shoulder, T_base_sh)) return false;                                   // TF disponibile?
    out_pos = T_base_sh.block<3,1>(0,3);                                                                // prende solo la traslazione (colonna 3)
    return true;                                                                                        // ok
}

Eigen::Quaterniond ExecuteDanceComponent::quatAlignAxisToDir(
    const Eigen::Vector3d& dir_base,
    ToolAxis /*axis*/,                             // ignorato: lavoriamo sempre in AlignX
    const Eigen::Vector3d& worldUp                 // es. UnitZ o UnitY a seconda della tua convenzione
) const
{
    // 1) Asse X dell'EEF puntato verso il target
    Eigen::Vector3d fwd = dir_base.normalized();   // X = fwd

    // 2) Scegli un "up" non parallelo a fwd (robustezza numerica)
    Eigen::Vector3d up = worldUp.normalized();
    if (std::abs(fwd.dot(up)) > 0.999) {
        // se quasi paralleli, scegli un up alternativo
        up = (std::abs(fwd.dot(Eigen::Vector3d::UnitZ())) < 0.999)
               ? Eigen::Vector3d::UnitZ()
               : Eigen::Vector3d::UnitY();
    }

    // 3) Completa la terna ortonormale via cross products (stile Gram–Schmidt)
    Eigen::Vector3d right = up.cross(fwd).normalized(); // Z = right (o asse 3)
    up = fwd.cross(right).normalized();                  // Y ricalcolata ortogonale

    // 4) Costruisci R con colonne = assi del tool frame (X=fwd, Y=up, Z=right)
    Eigen::Matrix3d R;
    R.col(0) = fwd;     // X → target
    R.col(1) = up;      // Y ortogonale (più vicino a worldUp possibile)
    R.col(2) = right;   // Z ortogonale

    return Eigen::Quaterniond(R);
}



Eigen::Vector3d ExecuteDanceComponent::sphereReachPoint(const Eigen::Vector3d& shoulder_base,
                                                        const Eigen::Vector3d& target_base) const
{
    // Punto sulla retta spalla→target ad una distanza L (clippata) dalla spalla
    const double d = (target_base - shoulder_base).norm();
    const double R = std::min(m_reachRadius, std::max(d, m_minDist)); // clamp del raggio
    const double L = std::min(std::max(d - m_safetyBackoff, 0.0), R); // arretra un po' dal target
    if (d < 1e-6) return shoulder_base;
    const Eigen::Vector3d dir = (target_base - shoulder_base) / d;
    return shoulder_base + L * dir;
}
