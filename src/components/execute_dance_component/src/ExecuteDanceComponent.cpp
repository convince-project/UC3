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
    // ------------------------------------------------------------
    // request: contenente request->dance_name (nome dell'opera da puntare)
    // ------------------------------------------------------------
    RCLCPP_INFO(m_node->get_logger(), "EXECUTE TASK: starting executeTask for '%s'", request->dance_name.c_str());
    auto artworkIt = m_artworkCoords.find(request->dance_name);
    if (artworkIt == m_artworkCoords.end()) {
        // Se non troviamo l'opera terminiamo subito
        RCLCPP_WARN(m_node->get_logger(), "No ARTWORK found '%s', aborting executeTask.", request->dance_name.c_str());
        return;
    }

    // artwork_x/y/z: coordinate (map frame) dell'opera target
    const double artwork_x = artworkIt->second[0];
    const double artwork_y = artworkIt->second[1];
    const double artwork_z = artworkIt->second[2];
    RCLCPP_INFO(m_node->get_logger(), "ARTWORK coords: '%s' [%.3f, %.3f, %.3f]",
                request->dance_name.c_str(), artwork_x, artwork_y, artwork_z);

    // ------------------------------------------------------------
    // PRE-SCAN: raccolgo pose attuali di entrambi i bracci e calcolo
    // la distanza euclidea opera <-> mano. Salvo le pose "flattened"
    // in cachedPoseValues per riuso nella fase di esecuzione.
    // - armDistances: vettore di (nome_braccio, distanza)
    // - cachedPoseValues: map da nome_braccio -> vector<double> flattenato
    // ------------------------------------------------------------
    std::vector<std::pair<std::string,double>> armDistances;                 // (armName, distance_to_artwork)
    std::map<std::string, std::vector<double>> cachedPoseValues;            // armName -> flat pose vector

    for (const std::string armId : {"LEFT", "RIGHT"}) {
        // clientPort: porta YARP locale usata per inviare comandi RPC al controller del braccio
        yarp::os::Port* clientPort = (armId == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string serverPort = (armId == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                        : "/r1-cartesian-control/right_arm/rpc:i";

        // verifico connettività prima di chiedere la posa
        if (!clientPort->isOpen() || !yarp::os::Network::isConnected(clientPort->getName(), serverPort)) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: %s port not ready, skipping", armId.c_str());
            continue;
        }

        // chiedo la posa corrente al controller: get_pose -> res_get
        yarp::os::Bottle cmd_get, res_get;
        cmd_get.addString("get_pose");
        bool write_ok = clientPort->write(cmd_get, res_get);
        RCLCPP_INFO(m_node->get_logger(), "PRE-SCAN: %s get_pose write_ok=%d res_size=%ld", armId.c_str(), write_ok, res_get.size());
        if (!write_ok) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: get_pose failed for %s", armId.c_str());
            continue;
        }

        // Flatten della Bottle in vector<double> (risposta YARP può avere sotto-liste)
        std::vector<double> flat_pose;
        for (size_t i = 0; i < res_get.size(); ++i) {
            if (res_get.get(i).isList()) {
                yarp::os::Bottle* sub = res_get.get(i).asList();
                for (size_t j = 0; j < sub->size(); ++j) flat_pose.push_back(sub->get(j).asFloat64());
            } else {
                flat_pose.push_back(res_get.get(i).asFloat64());
            }
        }

        // Convenzione: la risposta può essere 16 (4x4) o 18 (2 header + 16). Qui controlliamo.
        if (flat_pose.size() != 18 && flat_pose.size() != 16) {
            RCLCPP_WARN(m_node->get_logger(), "PRE-SCAN: unexpected pose size=%zu for %s, skipping", flat_pose.size(), armId.c_str());
            continue;
        }

        // pose_offset: se la lista contiene 2 header, gli elementi reali della 4x4 iniziano a index=2
        const size_t pose_offset = (flat_pose.size() == 18) ? 2u : 0u;

        // Indici row-major della matrice 4x4 flattenata (riga-major):
        // r0: [0,1,2,3], r1: [4,5,6,7], r2: [8,9,10,11], r3: [12,13,14,15]
        // la traslazione è nella 4ª colonna: tx=index 3, ty=index 7, tz=index 11
        const double hand_tx = flat_pose[pose_offset + 3];
        const double hand_ty = flat_pose[pose_offset + 7];
        const double hand_tz = flat_pose[pose_offset + 11];

        // distanza euclidea opera <-> mano usata per scegliere il braccio più vicino
        const double euclidean_dist = std::hypot(std::hypot(artwork_x - hand_tx, artwork_y - hand_ty), artwork_z - hand_tz);
        RCLCPP_INFO(m_node->get_logger(), "PRE-SCAN: %s hand pos=[%.3f, %.3f, %.3f] dist=%.3f",
                    armId.c_str(), hand_tx, hand_ty, hand_tz, euclidean_dist);

        armDistances.emplace_back(armId, euclidean_dist);
        cachedPoseValues.emplace(armId, std::move(flat_pose)); // salvo la risposta flattenata per riuso
    }

    if (armDistances.empty()) {
        RCLCPP_ERROR(m_node->get_logger(), "No available arms to perform pointing. Aborting executeTask.");
        return;
    }

    // ordino per distanza crescente: provo prima il braccio più vicino
    std::sort(armDistances.begin(), armDistances.end(),
              [](const auto &a, const auto &b){ return a.second < b.second; });

    // ------------------------------------------------------------
    // EXECUTION: per ogni braccio ordinato provo a calcolare
    // l'orientamento target e a inviare is_pose_reachable / go_to_pose
    // ------------------------------------------------------------
    for (const auto &armEntry : armDistances) {
        const std::string armName = armEntry.first;
        RCLCPP_INFO(m_node->get_logger(), "ARM LOOP: trying arm '%s' (closest first)", armName.c_str());

        // activePort: porta YARP da usare per il braccio corrente
        yarp::os::Port* activePort = (armName == "LEFT") ? &m_cartesianPortLeft : &m_cartesianPortRight;
        const std::string remotePort = (armName == "LEFT") ? "/r1-cartesian-control/left_arm/rpc:i"
                                                           : "/r1-cartesian-control/right_arm/rpc:i";
        // ricontrollo connettività
        if (!activePort->isOpen() || !yarp::os::Network::isConnected(activePort->getName(), remotePort)) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: port not ready at execution time, skipping", armName.c_str());
            continue;
        }

        // recupero la pose flattenata dalla cache (evito di ricontattare il controller)
        auto it_cached = cachedPoseValues.find(armName);
        if (it_cached == cachedPoseValues.end()) {
            RCLCPP_WARN(m_node->get_logger(), "No cached pose for %s, skipping", armName.c_str());
            continue;
        }
        const std::vector<double>& flat_pose = it_cached->second; // const ref per evitare copie
        const size_t pose_offset = (flat_pose.size() == 18) ? 2u : 0u;
        if (flat_pose.size() < pose_offset + 12) { // safety: servono almeno 12 elementi dopo offset
            RCLCPP_WARN(m_node->get_logger(), "Cached pose too small for %s (size=%zu)", armName.c_str(), flat_pose.size());
            continue;
        }

        // -----------------------
        // Estrazione posizione mano
        // -----------------------
        // usiamo gli indici row-major visti prima
        const double hand_x = flat_pose[pose_offset + 3];
        const double hand_y = flat_pose[pose_offset + 7];
        const double hand_z = flat_pose[pose_offset + 11];
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: hand pos (cached) = [%.3f, %.3f, %.3f]", armName.c_str(), hand_x, hand_y, hand_z);

        // -----------------------
        // Estrazione orientamento mano (3x3 rotation matrix)
        // -----------------------
        // Mappiamo il blocco 4x4 (row-major) direttamente su Eigen::Matrix, poi prendiamo top-left 3x3.
        // Questo assume esplicitamente row-major (coerente coi log e con i dati ricevuti).
        Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>> T_row(flat_pose.data() + pose_offset);
        const Eigen::Matrix3d R_hand = T_row.topLeftCorner<3,3>(); // orientamento della mano espresso nel frame mondo

        // ------------------------------------------------------------
        // Calcolo vettore direzione dalla mano all'opera e normalizzazione    
        // Matematica:
        //   v = p_art - p_hand
        //   ||v|| = sqrt(v_x^2 + v_y^2 + v_z^2)
        //   u = v / ||v||   (se ||v|| > eps)
        // Uso:
        //   u è vettore unitario che indica solo direzione (non dipende dalla distanza)
        // ------------------------------------------------------------
        const Eigen::Vector3d hand_pos(hand_x, hand_y, hand_z);
        const Eigen::Vector3d artwork_pos(artwork_x, artwork_y, artwork_z);
        Eigen::Vector3d vec_to_artwork = artwork_pos - hand_pos;      // v = p_art - p_hand
        const double vec_norm = vec_to_artwork.norm();               // ||v||
        if (vec_norm <= 1e-9) {
            // vettore troppo piccolo (posizione identica o numericamente nulla) -> non ha senso orientare
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: degenerate vector from hand to artwork (norm=%.6e), skipping", armName.c_str(), vec_norm);
            continue;
        }
        vec_to_artwork.normalize(); // u = v / ||v||  ; ora vec_to_artwork è unitario
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: vec_hand_to_artwork = [%.4f, %.4f, %.4f] (orig_norm=%.6f)",
                    armName.c_str(), vec_to_artwork.x(), vec_to_artwork.y(), vec_to_artwork.z(), vec_norm);

        // --- DEBUG EXTRA: punto verso cui viene orientata la mano (usato per calcolare q_target)
        // Questo è il punto dell'artwork nel world/map frame usato per definire la direzione di pointing.
        RCLCPP_INFO(m_node->get_logger(),
                    "ARM %s: orientation target point (artwork) = [%.3f, %.3f, %.3f]",
                    armName.c_str(), artwork_pos.x(), artwork_pos.y(), artwork_pos.z());
        
        // ------------------------------------------------------------
        // Determino la rotazione minima che allinea l'asse locale X della mano
        // con il vettore unitario vec_to_artwork:
        // - hand_local_x: prima colonna di R_hand = asse X locale espresso nel mondo
        // - cos_angle = dot(hand_local_x, vec_to_artwork)  -> coseno dell'angolo tra i due vettori
        // - rotation_axis = cross(hand_local_x, vec_to_artwork) -> asse di rotazione (non normalizzato)
        // - axis_norm = ||rotation_axis|| ; se vicino a 0 vettori paralleli/antiparalleli
        // ------------------------------------------------------------
        const Eigen::Vector3d hand_local_x = R_hand.col(0); // prima colonna = asse X locale della mano
        const double cos_angle = std::clamp(hand_local_x.dot(vec_to_artwork), -1.0, 1.0); // clamp per sicurezza numerica
        Eigen::Vector3d rotation_axis = hand_local_x.cross(vec_to_artwork); // asse non normalizzato
        double axis_norm = rotation_axis.norm();

        Eigen::Quaterniond q_target; // orientamento target rappresentato come quaternion

        if (axis_norm <= 1e-6) {
            // casi limite: vettori quasi paralleli o antiparalleli
            if (cos_angle > 0.999999) {
                // praticamente già allineati: mantengo l'orientamento corrente
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: already aligned (no rotation required)", armName.c_str());
                q_target = Eigen::Quaterniond(R_hand);
            } else {
                // antiparalleli: dot ≈ -1 -> serve rotazione di ~180°
                // cross = 0 quindi scegliamo un asse ortogonale arbitrario e ruotiamo di PI
                Eigen::Vector3d fallback = (std::abs(hand_local_x.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
                rotation_axis = hand_local_x.cross(fallback);
                if (rotation_axis.norm() <= 1e-9) rotation_axis = Eigen::Vector3d::UnitZ(); // ultima risorsa
                rotation_axis.normalize();
                const double angle = M_PI; // 180 gradi
                Eigen::AngleAxisd aa(angle, rotation_axis);
                // Left-multiply: R_target = aa * R_hand applica la rotazione nello spazio del mondo
                Eigen::Matrix3d R_target = aa * R_hand;
                q_target = Eigen::Quaterniond(R_target);
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: antiparallel case, rotating 180deg around [%f,%f,%f]",
                            armName.c_str(), rotation_axis.x(), rotation_axis.y(), rotation_axis.z());
            }
        } else {
            // normale caso: otteniamo l'angolo con acos(dot) e normalizziamo l'asse
            rotation_axis.normalize();
            const double angle = std::acos(cos_angle); // angolo minimale per allineare i due vettori
            Eigen::AngleAxisd aa(angle, rotation_axis);
            Eigen::Matrix3d R_target = aa * R_hand; // left-multiply -> rotazione nello spazio mondo
            q_target = Eigen::Quaterniond(R_target);
            RCLCPP_INFO(m_node->get_logger(), "ARM %s: rotating angle=%.4f around axis=[%.4f,%.4f,%.4f]",
                        armName.c_str(), angle, rotation_axis.x(), rotation_axis.y(), rotation_axis.z());
        }

        // --- DEBUG: log orientamenti (hand current + target) vicino al loro calcolo
        Eigen::Quaterniond q_current(R_hand); // orientamento corrente della mano
        RCLCPP_INFO(m_node->get_logger(),
                    "ARM %s: q_current (hand) = [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                    armName.c_str(), q_current.x(), q_current.y(), q_current.z(), q_current.w());
        RCLCPP_INFO(m_node->get_logger(),
                    "ARM %s: q_target (desired) = [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                    armName.c_str(), q_target.x(), q_target.y(), q_target.z(), q_target.w());

        // ------------------------------------------------------------
        // Check reachability e invio comando di posizionamento
        // - is_pose_reachable: ora controlliamo la REACHABILITY del PUNTO TARGET
        //   (artwork) con l'orientamento che vogliamo ottenere (q_target).
        // ------------------------------------------------------------
        yarp::os::Bottle cmd_check, res_check;
        cmd_check.addString("is_pose_reachable");
        // <-- changed: use target artwork coordinates (non la posizione corrente della mano)
        cmd_check.addFloat64(artwork_x);
        cmd_check.addFloat64(artwork_y);
        cmd_check.addFloat64(artwork_z);
        cmd_check.addFloat64(q_target.x());
        cmd_check.addFloat64(q_target.y());
        cmd_check.addFloat64(q_target.z());
        cmd_check.addFloat64(q_target.w());
        bool ok_check = activePort->write(cmd_check, res_check);
        RCLCPP_INFO(m_node->get_logger(), "ARM %s: is_pose_reachable (ARTWORK pt) write_ok=%d res_size=%ld",
                    armName.c_str(), ok_check, res_check.size());
        if (!ok_check || res_check.size() == 0 || res_check.get(0).asVocab32() != yarp::os::createVocab32('o','k')) {
            RCLCPP_WARN(m_node->get_logger(), "ARM %s: target artwork pose not reachable, trying next arm", armName.c_str());
            continue;
        }

        // ------------------------------------------------------------
        // Find the farthest reachable point along the ray hand -> artwork
        // (binary search assuming monotonicity: if p(t) è raggiungibile allora p(t') per t' <= t lo è)
        // ------------------------------------------------------------
        {
            const double safety_margin = 0.05;    // lascia un margine dall'opera (m)
            const double pos_tol = 1e-3;          // tolleranza posizione per terminare (m)
            const int    max_iters = 20;          // massimo iterazioni di ricerca binaria

            // d = distanza e direzione (vec_to_artwork è già normalizzato, vec_norm è la distanza)
            double max_t = std::max(0.0, vec_norm - safety_margin); // distanza massima da mano verso opera da provare
            if (max_t <= 0.0) {
                RCLCPP_WARN(m_node->get_logger(), "ARM %s: artwork troppo vicino (dist=%.6f), proveremo la posizione della mano", armName.c_str(), vec_norm);
                max_t = 0.0;
            }

            // lambda che domanda is_pose_reachable per una posizione candidata con q_target
            auto isReachableCandidate = [&](const Eigen::Vector3d& candidate)->bool {
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
                    RCLCPP_WARN(m_node->get_logger(), "ARM %s: RPC error on is_pose_reachable for candidate [%.3f,%.3f,%.3f]",
                                armName.c_str(), candidate.x(), candidate.y(), candidate.z());
                    return false;
                }
                return res_check_local.get(0).asVocab32() == yarp::os::createVocab32('o','k');
            };

            // --- DEBUG: log orientamenti e posizione mano prima di procedere
            Eigen::Quaterniond q_current(R_hand); // orientamento attuale della mano
            RCLCPP_INFO(m_node->get_logger(),
                        "ARM %s: q_current (hand) = [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                        armName.c_str(), q_current.x(), q_current.y(), q_current.z(), q_current.w());
            RCLCPP_INFO(m_node->get_logger(),
                        "ARM %s: q_target (desired) = [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                        armName.c_str(), q_target.x(), q_target.y(), q_target.z(), q_target.w());
            RCLCPP_INFO(m_node->get_logger(),
                        "ARM %s: hand_pos = [%.3f, %.3f, %.3f], artwork_pos = [%.3f, %.3f, %.3f], vec_norm=%.6f",
                        armName.c_str(), hand_pos.x(), hand_pos.y(), hand_pos.z(),
                        artwork_pos.x(), artwork_pos.y(), artwork_pos.z(), vec_norm);

            // Nota: non scartiamo l'arto se is_pose_reachable(hand_pos,q_current) fallisce,
            // perchè in alcuni casi il controller può rispondere "not reachable" per la posa attuale.
            // Procediamo direttamente a probing/binary search lungo la retta con q_target.

            // se il punto max (vicino all'opera) è raggiungibile prendi direttamente quello
            Eigen::Vector3d candidate_max = hand_pos + vec_to_artwork * max_t;
            RCLCPP_INFO(m_node->get_logger(),
                        "ARM %s: probing artwork-proximal candidate pos=[%.3f, %.3f, %.3f] with q_target=[%.6f, %.6f, %.6f, %.6f]",
                        armName.c_str(),
                        candidate_max.x(), candidate_max.y(), candidate_max.z(),
                        q_target.x(), q_target.y(), q_target.z(), q_target.w());
            if (isReachableCandidate(candidate_max)) {
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: artwork-proximal candidate reachable (t=%.3f)", armName.c_str(), max_t);
                // manda go_to_pose direttamente su candidate_max
                yarp::os::Bottle cmd_pose_local, res_pose_local;
                cmd_pose_local.addString("go_to_pose");
                cmd_pose_local.addFloat64(candidate_max.x()); cmd_pose_local.addFloat64(candidate_max.y()); cmd_pose_local.addFloat64(candidate_max.z());
                cmd_pose_local.addFloat64(q_target.x()); cmd_pose_local.addFloat64(q_target.y()); cmd_pose_local.addFloat64(q_target.z()); cmd_pose_local.addFloat64(q_target.w());
                cmd_pose_local.addFloat64(15.0);
                bool ok_pose_local = activePort->write(cmd_pose_local, res_pose_local);
                RCLCPP_INFO(m_node->get_logger(), "ARM %s: go_to_pose (candidate_max) write_ok=%d res_size=%ld", armName.c_str(), ok_pose_local, res_pose_local.size());
                if (ok_pose_local && res_pose_local.size() > 0 && res_pose_local.get(0).asVocab32() == yarp::os::createVocab32('o','k')) {
                    RCLCPP_INFO(m_node->get_logger(), "POSITIONING SUCCESS at candidate_max for %s", armName.c_str());
                    break;
                } else {
                    RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed for candidate_max, trying binary search", armName.c_str());
                    // fallthrough: proveremo binary search
                }
            }

            // Binary search su t in [0, max_t] per trovare il valore massimo t raggiungibile
            double lo = 0.0;
            double hi = max_t;
            double best_t = 0.0;
            for (int it = 0; it < max_iters && (hi - lo) > pos_tol; ++it) {
                double mid = 0.5 * (lo + hi);
                Eigen::Vector3d candidate = hand_pos + vec_to_artwork * mid;

                // DEBUG: log ogni iterazione (utile per tracciare i probe)
                RCLCPP_DEBUG(m_node->get_logger(), "ARM %s: binary iter=%d mid=%.6f candidate=[%.3f,%.3f,%.3f]",
                             armName.c_str(), it, mid, candidate.x(), candidate.y(), candidate.z());

                if (isReachableCandidate(candidate)) {
                    best_t = mid;
                    lo = mid;
                    RCLCPP_DEBUG(m_node->get_logger(), "ARM %s: candidate reachable at mid=%.6f", armName.c_str(), mid);
                } else {
                    hi = mid;
                    RCLCPP_DEBUG(m_node->get_logger(), "ARM %s: candidate NOT reachable at mid=%.6f", armName.c_str(), mid);
                }
            }

            Eigen::Vector3d best_candidate = hand_pos + vec_to_artwork * best_t;
            RCLCPP_INFO(m_node->get_logger(),
                        "ARM %s: selected candidate t=%.4f -> pos=[%.3f, %.3f, %.3f] ; q_target=[%.6f, %.6f, %.6f, %.6f]",
                        armName.c_str(),
                        best_t, best_candidate.x(), best_candidate.y(), best_candidate.z(),
                        q_target.x(), q_target.y(), q_target.z(), q_target.w());

            // --- DEBUG EXTRA: ripetiamo esplicitamente il punto verso cui si è orientato (artwork)
            RCLCPP_INFO(m_node->get_logger(),
                        "ARM %s: orientation target (artwork) = [%.3f, %.3f, %.3f], final commanded pos = [%.3f, %.3f, %.3f]",
                        armName.c_str(),
                        artwork_pos.x(), artwork_pos.y(), artwork_pos.z(),
                        best_candidate.x(), best_candidate.y(), best_candidate.z());

            // ultima verifica e invio go_to_pose su best_candidate
            yarp::os::Bottle cmd_pose_final, res_pose_final;
            cmd_pose_final.addString("go_to_pose");
            cmd_pose_final.addFloat64(best_candidate.x()); cmd_pose_final.addFloat64(best_candidate.y()); cmd_pose_final.addFloat64(best_candidate.z());
            cmd_pose_final.addFloat64(q_target.x()); cmd_pose_final.addFloat64(q_target.y()); cmd_pose_final.addFloat64(q_target.z()); cmd_pose_final.addFloat64(q_target.w());
            cmd_pose_final.addFloat64(15.0);
            bool ok_pose_final = activePort->write(cmd_pose_final, res_pose_final);
            RCLCPP_INFO(m_node->get_logger(), "ARM %s: go_to_pose (final candidate) write_ok=%d res_size=%ld", armName.c_str(), ok_pose_final, res_pose_final.size());
            if (ok_pose_final && res_pose_final.size() > 0 && res_pose_final.get(0).asVocab32() == yarp::os::createVocab32('o','k')) {
                RCLCPP_INFO(m_node->get_logger(), "POSITIONING SUCCESS: %s hand aligned toward artwork [%s]", armName.c_str(), request->dance_name.c_str());
                break; // successo: non provare l'altro braccio
            } else {
                RCLCPP_WARN(m_node->get_logger(), "ARM %s: go_to_pose failed for final candidate, trying next arm", armName.c_str());
                continue;
            }
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

