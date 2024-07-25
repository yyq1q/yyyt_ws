// -*- C++ -*-
// <rtc-template block="description">
/*!
 * @file  raspai2shougaibutsuTest.cpp
 * @brief roboto controller (test code)
 *
 */
// </rtc-template>

#include "raspai2shougaibutsuTest.h"

// Module specification
// <rtc-template block="module_spec">
#if RTM_MAJOR_VERSION >= 2
static const char* const raspai2shougaibutsu_spec[] =
#else
static const char* raspai2shougaibutsu_spec[] =
#endif
  {
    "implementation_id", "raspai2shougaibutsuTest",
    "type_name",         "raspai2shougaibutsuTest",
    "description",       "roboto controller",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
raspai2shougaibutsuTest::raspai2shougaibutsuTest(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_inOut("in", m_in),
    m_sensorOut("sensor", m_sensor),
    m_outIn("out", m_out),
    m_endIn("end", m_end)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
raspai2shougaibutsuTest::~raspai2shougaibutsuTest()
{
}



RTC::ReturnCode_t raspai2shougaibutsuTest::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("out", m_outIn);
  addInPort("end", m_endIn);
  
  // Set OutPort buffer
  addOutPort("in", m_inOut);
  addOutPort("sensor", m_sensorOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t raspai2shougaibutsuTest::onFinalize()
{
  return RTC::RTC_OK;
}
*/


//RTC::ReturnCode_t raspai2shougaibutsuTest::onStartup(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsuTest::onShutdown(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


RTC::ReturnCode_t raspai2shougaibutsuTest::onActivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t raspai2shougaibutsuTest::onDeactivated(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t raspai2shougaibutsuTest::onExecute(RTC::UniqueId /*ec_id*/)
{
  return RTC::RTC_OK;
}


//RTC::ReturnCode_t raspai2shougaibutsuTest::onAborting(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsuTest::onError(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsuTest::onReset(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsuTest::onStateUpdate(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


//RTC::ReturnCode_t raspai2shougaibutsuTest::onRateChanged(RTC::UniqueId /*ec_id*/)
//{
//  return RTC::RTC_OK;
//}


bool raspai2shougaibutsuTest::runTest()
{
    return true;
}


extern "C"
{
 
  void raspai2shougaibutsuTestInit(RTC::Manager* manager)
  {
    coil::Properties profile(raspai2shougaibutsu_spec);
    manager->registerFactory(profile,
                             RTC::Create<raspai2shougaibutsuTest>,
                             RTC::Delete<raspai2shougaibutsuTest>);
  }
  
}
