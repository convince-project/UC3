#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <nlohmann/json.hpp> // Include the JSON library (https://github.com/nlohmann/json)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "text_to_speech_interfaces/action/batch_generation.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Sound.h>
#include <yarp/sig/SoundFile.h> // for yarp::sig::file::write_wav_file

using namespace std::chrono_literals;
using json = nlohmann::json;
using BatchGen = text_to_speech_interfaces::action::BatchGeneration;

bool process_text_to_speech(const std::string &text, const std::string &out_wav, rclcpp::Node::SharedPtr node, rclcpp_action::Client<BatchGen>::SharedPtr action_client) {
    // Prepare and send goal
    BatchGen::Goal goal;
    goal.texts = { text };

    RCLCPP_INFO(node->get_logger(), "Sending goal to BatchGenerationAction...");
    auto send_goal_future = action_client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, send_goal_future, 5s) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
        return false;
    }
    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        return false;
    }
    RCLCPP_INFO(node->get_logger(), "Goal accepted, waiting for result...");

    // Request result asynchronously
    auto result_future = action_client->async_get_result(goal_handle);

    // --- YARP init and open input port ---
    yarp::os::Network yarp;
    if (!yarp.checkNetwork(2.0)) {
        RCLCPP_ERROR(node->get_logger(), "YARP network not available (is yarpserver running?)");
        return false;
    }

    yarp::os::BufferedPort<yarp::sig::Sound> inport;
    std::string local_port = "/batch_client_cpp/in";
    if (!inport.open(local_port)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open YARP input port %s", local_port.c_str());
        return false;
    }

    // Try to connect remote -> local
    std::string remote = "/TextToSpeechComponent/batch:o";
    bool connected = yarp::os::Network::connect(remote, local_port);
    if (!connected) {
        RCLCPP_WARN(node->get_logger(), "Unable to connect %s -> %s", remote.c_str(), local_port.c_str());
    } else {
        RCLCPP_INFO(node->get_logger(), "Connected %s -> %s", remote.c_str(), local_port.c_str());
    }

    // Wait for sound on the inport with timeout
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = 40s;
    yarp::sig::Sound* sound = nullptr;

    RCLCPP_INFO(node->get_logger(), "Waiting for sound on YARP port %s ...", local_port.c_str());
    while (std::chrono::steady_clock::now() - start < timeout) {
        sound = inport.read(false);
        if (sound != nullptr && sound->getSamples() > 0) {
            RCLCPP_INFO(node->get_logger(), "Received sound: samples=%d channels=%d", sound->getSamples(), sound->getChannels());
            break;
        }
        std::this_thread::sleep_for(200ms);
        rclcpp::spin_some(node);
    }

    if (sound == nullptr || sound->getSamples() == 0) {
        RCLCPP_ERROR(node->get_logger(), "Timeout waiting for sound on YARP port");
        inport.close();
        yarp::os::Network::fini();
        return false;
    }

    // Write WAV using yarp::sig::file::write_wav_file
    try {
        bool ok = yarp::sig::file::write(*sound, out_wav.c_str());
        if (!ok) {
            RCLCPP_ERROR(node->get_logger(), "Failed to write wav file %s", out_wav.c_str());
            inport.close();
            yarp::os::Network::fini();
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "Saved WAV to %s", out_wav.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Exception while writing wav: %s", e.what());
        inport.close();
        yarp::os::Network::fini();
        return false;
    }

    inport.close();
    yarp::os::Network::fini();
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_json>\n";
        return 1;
    }
    std::string json_path = argv[1];

    // Parse JSON file
    std::ifstream json_file(json_path);
    if (!json_file.is_open()) {
        std::cerr << "Failed to open JSON file: " << json_path << "\n";
        return 1;
    }
    json data;
    json_file >> data;

    // --- ROS2 init ---
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("batch_client_cpp");
    auto action_client = rclcpp_action::create_client<BatchGen>(node, "/TextToSpeechComponent/BatchGenerationAction");

    // Wait for action server
    if (!action_client->wait_for_action_server(10s)) {
        RCLCPP_ERROR(node->get_logger(), "BatchGenerationAction server not available after 10s");
        rclcpp::shutdown();
        return 2;
    }

    // Process "it-IT" section
    auto it_it = data["TOUR_MADAMA_3"]["m_availablePoIs"]["it-IT"];
    for (auto& [key, value] : it_it.items()) {
        if (value.contains("m_availableActions")) {
            for (auto& [action_name, actions] : value["m_availableActions"].items()) {
                std::string concatenated_text;
                for (auto& action : actions) {
                    if (action.contains("m_param")) {
                        concatenated_text += action["m_param"].get<std::string>() + " ";
                    }
                }
                if (!concatenated_text.empty()) {
                    std::string out_wav = "/home/user1/UC3/conf/audio/male/" + key + "_" + action_name + ".wav";
                    RCLCPP_INFO(node->get_logger(), "Processing text: %s -> %s", concatenated_text.c_str(), out_wav.c_str());
                    if (!process_text_to_speech(concatenated_text, out_wav, node, action_client)) {
                        RCLCPP_ERROR(node->get_logger(), "Failed to process text: %s", concatenated_text.c_str());
                    }
                }
            }
        }
    }

    rclcpp::shutdown();
    return 0;
}