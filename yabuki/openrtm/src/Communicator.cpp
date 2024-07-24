// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  Communicator.cpp
 * @brief ModuleDescription
 *
 */
// </rtc-template>

#include "Communicator.h"
#include <zmq.hpp>
#include <include/zmq.hpp>
#include <include/zmq_addon.hpp>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

float vx = 0.0, vy = 0.0, vz = 0.0;

//void subscriber() {
//    zmq::context_t context(1);
//    zmq::socket_t socket(context, ZMQ_SUB);
//    socket.connect("tcp://192.168.0.105:7777");
//    socket.set(zmq::sockopt::subscribe, "");
//
//    while (true) {
//        zmq::message_t message;
//        //socket.recv(message, zmq::recv_flags::none);
//        socket.recv(message);
//        std::string received_message(static_cast<char*>(message.data()), message.size());
//
//        if (received_message == "Hello, ZeroMQ!") {
//            std::cout << "Received text message: " << received_message << std::endl;
//        }
//        else {
//            // 配列データの処理
//            float* arr = static_cast<float*>(message.data());
//            size_t arr_size = message.size() / sizeof(int);
//            //std::cout << arr_size << std::endl;
//
//            std::cout << "Received array: ";
//            for (size_t i = 0; i < arr_size; ++i) {
//                std::cout << arr[i] << " ";
//            }
//            std::cout << std::endl;
//            vx = arr[0];
//            vy = arr[1];
//            vz = arr[2];
//        }
//    }
//}

void subscriber()
{
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_SUB);
    socket.bind("tcp://*:7777");
    socket.set(zmq::sockopt::subscribe, "");
    // バッファリングの最適化
    //socket.set(zmq::sockopt::rcvhwm, 12);
    //socket.set(zmq::sockopt::rcvbuf, 12);
    //socket.set(zmq::sockopt::rcvtimeo, 100);

    zmq::message_t message;
    size_t arr_size = 0;
    if(socket.recv(message))
    {
        std::cout << "Received array: " << message.size() << std::endl;;
        float* arr = static_cast<float*>(message.data());
        arr_size = message.size() / sizeof(float);
        if (arr_size == 3)
        {
            for (size_t i = 0; i < arr_size; ++i)
            {
                std::cout << arr[i] << " ";
            }
            std::cout << std::endl;
            vx = arr[0];
            vy = arr[1];
            vz = arr[2];
        }
        else
        {
            std::cout << "ERROR" << std::endl;
            vx = 0.0;
            vy = 0.0;
            vz = 0.0;
        }
    }
    else
    {
        std::cout << "WARN" << std::endl;
        /*vx = 0.0;
        vy = 0.0;
        vz = 0.0;*/
    }
    /*try
    {
        if (socket.recv(message))
        {
            std::cout << "Received array: " << message.size() << std::endl;;
            float* arr = static_cast<float*>(message.data());
            arr_size = message.size() / sizeof(float);
            if (arr_size == 3)
            {
                for (size_t i = 0; i < arr_size; ++i)
                {
                    std::cout << arr[i] << " ";
                }
                std::cout << std::endl;
                vx = arr[0];
                vy = arr[1];
                vz = arr[2];
            }
            else
            {
                vx = 0.0;
                vy = 0.0;
                vz = 0.0;
            }
        }
        else
        {
            vx = 0.0;
            vy = 0.0;
            vz = 0.0;
        }
    }
    catch (const zmq::error_t& e)
    {
        std::cerr << "ZeroMQ error: " << e.what() << std::endl;
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        return;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        return;
    }
    catch (...)
    {
        std::cerr << "Unknown error occurred." << std::endl;
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        return;
    }*/
}

//void subscriber()
//{
//    zmq::context_t context(1);
//    zmq::socket_t socket(context, ZMQ_SUB);
//    socket.bind("tcp://*:7777");
//    socket.set(zmq::sockopt::subscribe, "");
//    zmq::message_t message;
//    size_t arr_size = 0;
//    try
//    {
//        while (arr_size < 3) {
//            socket.recv(message);
//            float* arr = static_cast<float*>(message.data());
//            arr_size = message.size() / sizeof(float);
//            std::cout << "Received array:" << arr_size << std::endl;;
//            for (size_t i = 0; i < arr_size; ++i)
//            {
//                std::cout << arr[i] << " ";
//            }
//            std::cout << std::endl;
//            if (arr_size == 3)
//            {
//                vx = arr[0];
//                vy = arr[1];
//                vz = arr[2];
//            }
//            else
//            {
//                vx = 0.0;
//                vy = 0.0;
//                vz = 0.0;
//            }
//        }
//    }
//    catch (const zmq::error_t& e)
//    {
//        std::cerr << "ZeroMQ error : " << e.what() << std::endl;
//        return;
//    }
//    catch (const std::exception& e)
//    {
//        std::cerr << "Standard exception : " << e.what() << std::endl;
//        return;
//    }
//    catch (...)
//    {
//        std::cerr << "Unknown error occurred." << std::endl;
//        return;
//    }
//}

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const communicator_spec[] =
#else
static const char* communicator_spec[] =
#endif
  {
    "implementation_id", "Communicator",
    "type_name",         "Communicator",
    "description",       "ModuleDescription",
    "version",           "1.0.0",
    "vendor",            "yabu",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.speed_x", "0.0",
    "conf.default.speed_r", "0.0",

    // Widget
    "conf.__widget__.speed_x", "slider.0.01",
    "conf.__widget__.speed_r", "slider.0.01",
    // Constraints
    "conf.__constraints__.speed_x", "-1.5<x<1.5",
    "conf.__constraints__.speed_r", "-2.0<x<2.0",

    "conf.__type__.speed_x", "double",
    "conf.__type__.speed_r", "double",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
Communicator::Communicator(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_outOut("out", m_out)
    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
Communicator::~Communicator()
{
}



RTC::ReturnCode_t Communicator::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  addOutPort("out", m_outOut);

  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("speed_x", m_speed_x, "0.0");
  bindParameter("speed_r", m_speed_r, "0.0");
  // </rtc-template>

  std::cout << "onInitialize" << std::endl;
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t Communicator::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t Communicator::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Communicator::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t Communicator::onActivated(RTC::UniqueId /*ec_id*/)
{
    std::cout << "onActivated" << std::endl;

    m_out.data.vx = 0.0;
    m_out.data.va = 0.0;
    m_outOut.write();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t Communicator::onDeactivated(RTC::UniqueId /*ec_id*/)
{
    m_out.data.vx = 0.0;
    m_out.data.va = 0.0;
    m_outOut.write();
    return RTC::RTC_OK;
}


RTC::ReturnCode_t Communicator::onExecute(RTC::UniqueId /*ec_id*/)
{
    //std::cout << "onExecute" << std::endl;
    subscriber();

    m_out.data.vx = vx;
    m_out.data.va = vz;
    m_outOut.write();

    return RTC::RTC_OK;
}


//RTC::ReturnCode_t Communicator::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Communicator::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Communicator::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Communicator::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t Communicator::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}



extern "C"
{
 
  void CommunicatorInit(RTC::Manager* manager)
  {
    coil::Properties profile(communicator_spec);
    manager->registerFactory(profile,
                             RTC::Create<Communicator>,
                             RTC::Delete<Communicator>);
  }
  
}
