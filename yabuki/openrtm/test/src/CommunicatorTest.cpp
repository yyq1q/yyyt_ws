// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  CommunicatorTest.cpp
 * @brief ModuleDescription (test code)
 *
 */
// </rtc-template>

#include "CommunicatorTest.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const communicator_spec[] =
#else
static const char* communicator_spec[] =
#endif
  {
    "implementation_id", "CommunicatorTest",
    "type_name",         "CommunicatorTest",
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

    "conf.__type__.speed_x", "double",
    "conf.__type__.speed_r", "double",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
CommunicatorTest::CommunicatorTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_outIn("out", m_out)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
CommunicatorTest::~CommunicatorTest()
{
}



RTC::ReturnCode_t CommunicatorTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("out", m_outIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("speed_x", m_speed_x, "0.0");
  bindParameter("speed_r", m_speed_r, "0.0");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t CommunicatorTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t CommunicatorTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t CommunicatorTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t CommunicatorTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CommunicatorTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t CommunicatorTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t CommunicatorTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t CommunicatorTest::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t CommunicatorTest::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t CommunicatorTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t CommunicatorTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool CommunicatorTest::runTest()
{
    return true;
}


extern "C"
{
 
  void CommunicatorTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(communicator_spec);
    manager->registerFactory(profile,
                             RTC::Create<CommunicatorTest>,
                             RTC::Delete<CommunicatorTest>);
  }
  
}
