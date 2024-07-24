#include <iostream>
#include <zmq.hpp>
#include <zmq_addon.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <cstdio>

void publisher() {
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUB);
    socket.bind("tcp://*:7777");

    while (true) {
        std::cout << "Hello, World!" << std::endl;
        zmq::message_t message("Hello, World!", 13);
        socket.send(message, zmq::send_flags::none);

        int arr[] = {9, 8, 7, 6, 5};
        zmq::message_t message_array(sizeof(arr));
        memcpy(message_array.data(), arr, sizeof(arr));
        socket.send(message_array, zmq::send_flags::none);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main() {
    try {
        publisher();
    } catch (const zmq::error_t& e) {
        std::cerr << "ZeroMQ error: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred." << std::endl;
        return 1;
    }
    return 0;
}
